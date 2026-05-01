/**
 * @file rc522.cpp
 * @brief SPI RFID RC522 driver — ISO 14443A
 */

#include "rc522.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#define RC522_TIMEOUT_MS  25U

/*
 * RC522 address format: bit7=R/W, bits[6:1]=address, bit0=0
 * Read  : (reg << 1) | 0x80
 * Write : (reg << 1) & 0x7E
 */
#define SPI_ADDR_READ(reg)	(((reg) << 1U) | 0x80U)
#define SPI_ADDR_WRITE(reg)  (((reg) << 1U) & 0x7EU)

uint8_t rc522_read_reg(const rc522_handle_t *h, uint8_t reg)
{
	uint8_t val;

	SPI.beginTransaction(SPISettings(RC522_SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
	digitalWrite(h->pin_cs, LOW);
	SPI.transfer(SPI_ADDR_READ(reg));
	val = SPI.transfer(0x00U);
	digitalWrite(h->pin_cs, HIGH);
	SPI.endTransaction();

	return val;
}

void rc522_write_reg(const rc522_handle_t *h, uint8_t reg, uint8_t val)
{
	SPI.beginTransaction(SPISettings(RC522_SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
	digitalWrite(h->pin_cs, LOW);
	SPI.transfer(SPI_ADDR_WRITE(reg));
	SPI.transfer(val);
	digitalWrite(h->pin_cs, HIGH);
	SPI.endTransaction();
}

void rc522_set_bits(const rc522_handle_t *h, uint8_t reg, uint8_t mask)
{
	rc522_write_reg(h, reg, rc522_read_reg(h, reg) | mask);
}

void rc522_clear_bits(const rc522_handle_t *h, uint8_t reg, uint8_t mask)
{
	rc522_write_reg(h, reg, rc522_read_reg(h, reg) & ~mask);
}

static void rc522_hard_reset(const rc522_handle_t *h)
{
	digitalWrite(h->pin_rst, LOW);
	delayMicroseconds(150U);  /* datasheet §8.8 : minimum 100µs */
	digitalWrite(h->pin_rst, HIGH);
	delayMicroseconds(50U);
}

static void rc522_soft_reset(const rc522_handle_t *h)
{
	uint32_t t0;

	rc522_write_reg(h, RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);

	/* bit4 (PowerDown) clears to 0 when reset is complete */
	t0 = millis();
	while ((rc522_read_reg(h, RC522_REG_COMMAND) & 0x10U) != 0U) {
		if ((millis() - t0) > 50U) { break; }
	}
}

void rc522_init(rc522_handle_t *h, uint8_t pin_cs, uint8_t pin_rst)
{
	h->pin_cs	  = pin_cs;
	h->pin_rst	 = pin_rst;
	h->initialized = false;

	pinMode(h->pin_cs,  OUTPUT);
	pinMode(h->pin_rst, OUTPUT);
	digitalWrite(h->pin_cs,  HIGH);
	digitalWrite(h->pin_rst, HIGH);

	SPI.begin(RC522_PIN_SCK, RC522_PIN_MISO, RC522_PIN_MOSI, h->pin_cs);

	rc522_hard_reset(h);
	rc522_soft_reset(h);

	/* Auto timer — reload 0x03E8 at f_timer ≈ 40kHz → timeout ~25ms */
	rc522_write_reg(h, RC522_REG_T_MODE,	  0x80U);
	rc522_write_reg(h, RC522_REG_T_PRESCALER, 0xA9U);
	rc522_write_reg(h, RC522_REG_T_RELOAD_H,  0x03U);
	rc522_write_reg(h, RC522_REG_T_RELOAD_L,  0xE8U);

	rc522_write_reg(h, RC522_REG_TX_ASK, 0x40U);  /* 100% ASK modulation — ISO14443A */
	rc522_write_reg(h, RC522_REG_MODE,	0x3DU);  /* CRC preset 0x6363 — ISO14443A §6 */
	rc522_set_bits(h,  RC522_REG_TX_CONTROL, 0x03U);

	h->initialized = true;
}

uint8_t rc522_get_version(const rc522_handle_t *h)
{
	return rc522_read_reg(h, RC522_REG_VERSION);
}

/**
 * @brief Sends data through the FIFO and waits for the response.
 *
 * @param cmd		  RC522_CMD_TRANSCEIVE or RC522_CMD_MF_AUTHENT
 * @param tx_data	  Data to transmit
 * @param tx_len		Number of bytes to transmit
 * @param rx_data	  Receive buffer
 * @param rx_len		In: buffer capacity. Out: bytes received.
 * @param last_bits	Valid bits in the last received byte (0 = 8 bits)
 * @param tx_last_bits Valid bits to transmit in the last byte (0 = 8 bits)
 */
static rc522_status_t rc522_transceive(
	const rc522_handle_t *h,
	uint8_t				cmd,
	const uint8_t		*tx_data,
	uint8_t				tx_len,
	uint8_t			  *rx_data,
	uint8_t			  *rx_len,
	uint8_t			  *last_bits,
	uint8_t				tx_last_bits)
{
	uint8_t  wait_irq;
	uint8_t  irq_reg;
	uint8_t  err_reg;
	uint8_t  fifo_level;
	uint8_t  i;
	uint32_t t0;

	wait_irq = (cmd == RC522_CMD_MF_AUTHENT) ? 0x10U : 0x30U;

	rc522_write_reg(h, RC522_REG_COMMAND,	RC522_CMD_IDLE);
	rc522_write_reg(h, RC522_REG_COM_IRQ,	0x7FU);
	rc522_set_bits(h,  RC522_REG_FIFO_LEVEL, 0x80U);

	for (i = 0U; i < tx_len; i++)
		rc522_write_reg(h, RC522_REG_FIFO_DATA, tx_data[i]);

	rc522_write_reg(h, RC522_REG_BIT_FRAMING, tx_last_bits & 0x07U);
	rc522_write_reg(h, RC522_REG_COMMAND, cmd);

	if (cmd == RC522_CMD_TRANSCEIVE)
		rc522_set_bits(h, RC522_REG_BIT_FRAMING, 0x80U);

	t0 = millis();
	do
	{
		irq_reg = rc522_read_reg(h, RC522_REG_COM_IRQ);
		if ((irq_reg & wait_irq) != 0U)
			break;
		if ((irq_reg & 0x01U)	!= 0U)
			break;
		if ((millis() - t0) >= RC522_TIMEOUT_MS)
			break;
	} while (true);

	rc522_clear_bits(h, RC522_REG_BIT_FRAMING, 0x80U);

	if ((millis() - t0) >= RC522_TIMEOUT_MS)
		return RC522_ERR_TIMEOUT;
	if ((irq_reg & 0x01U) != 0U)
		return RC522_ERR_NOCARD;

	err_reg = rc522_read_reg(h, RC522_REG_ERROR);
	if ((err_reg & 0x13U) != 0U)
		return RC522_ERR_CRC;
	if ((err_reg & 0x08U) != 0U)
		return RC522_ERR_COLLISION;

	fifo_level = rc522_read_reg(h, RC522_REG_FIFO_LEVEL);

	if (last_bits != NULL)
		*last_bits = rc522_read_reg(h, RC522_REG_CONTROL) & 0x07U;

	if (rx_data != NULL && rx_len != NULL)
	{
		if (fifo_level > *rx_len)
			fifo_level = *rx_len;
		*rx_len = fifo_level;
		for (i = 0U; i < fifo_level; i++)
			rx_data[i] = rc522_read_reg(h, RC522_REG_FIFO_DATA);
	}

	return RC522_OK;
}

rc522_status_t rc522_is_card_present(const rc522_handle_t *h)
{
	uint8_t		cmd	= PICC_CMD_REQA;
	uint8_t		rx_buf[2];
	uint8_t		rx_len = sizeof(rx_buf);
	uint8_t		last_bits;
	rc522_status_t status;

	rc522_clear_bits(h, RC522_REG_COLL, 0x80U);

	/* REQA: short ISO14443A frame — 7 bits */
	status = rc522_transceive(h, RC522_CMD_TRANSCEIVE,
				&cmd, 1U,
				rx_buf, &rx_len,
				&last_bits, 7U);

	if (status != RC522_OK)
		return RC522_ERR_NOCARD;
	if (rx_len  != 2U)
		return RC522_ERR_NOCARD;

	return RC522_OK;
}

/**
 * @brief Computes CRC_A (ISO14443A) using the RC522 hardware coprocessor.
 */
static rc522_status_t rc522_calc_crc(const rc522_handle_t *h,
				const uint8_t *data, uint8_t len,
				uint8_t *crc_h, uint8_t *crc_l)
{
	uint8_t  i;
	uint32_t t0;

	rc522_write_reg(h, RC522_REG_COMMAND,	RC522_CMD_IDLE);
	rc522_write_reg(h, RC522_REG_DIV_IRQ,	0x04U);
	rc522_set_bits(h,  RC522_REG_FIFO_LEVEL, 0x80U);

	for (i = 0U; i < len; i++)
	{
		rc522_write_reg(h, RC522_REG_FIFO_DATA, data[i]);
	}
	rc522_write_reg(h, RC522_REG_COMMAND, RC522_CMD_CALC_CRC);

	t0 = millis();
	do
	{
		if ((rc522_read_reg(h, RC522_REG_DIV_IRQ) & 0x04U) != 0U)
			break;
		if ((millis() - t0) >= RC522_TIMEOUT_MS)
			return RC522_ERR_TIMEOUT;
	} while (true);

	rc522_write_reg(h, RC522_REG_COMMAND, RC522_CMD_IDLE);

	*crc_h = rc522_read_reg(h, RC522_REG_CRC_RESULT_H);
	*crc_l = rc522_read_reg(h, RC522_REG_CRC_RESULT_L);

	return RC522_OK;
}

/**
 * @brief Runs anticollision + select at one cascade level.
 *
 * Step 1 — anticollision: SEL + NVB=0x20, receives 4 UID bytes + BCC.
 * Step 2 — select		: SEL + NVB=0x70 + UID + BCC + CRC, receives SAK.
 */
static rc522_status_t rc522_anticoll_select(
	const rc522_handle_t *h,
	uint8_t				sel_cmd,
	uint8_t			  *uid_bytes,
	uint8_t			  *sak_out)
{
	uint8_t		tx_buf[9];
	uint8_t		rx_buf[5];
	uint8_t		rx_len;
	uint8_t		last_bits;
	uint8_t		bcc;
	uint8_t		i;
	uint8_t		crc_h;
	uint8_t		crc_l;
	rc522_status_t status;

	rc522_clear_bits(h, RC522_REG_COLL, 0x80U);

	tx_buf[0] = sel_cmd;
	tx_buf[1] = 0x20U;
	rx_len	= sizeof(rx_buf);

	status = rc522_transceive(h, RC522_CMD_TRANSCEIVE,
				tx_buf, 2U,
				rx_buf, &rx_len,
				&last_bits, 0U);

	if (status != RC522_OK) 
		return status;
	if (rx_len  != 5U)
		return RC522_ERR_NOCARD;

	/* BCC = XOR of the 4 UID bytes */
	bcc = rx_buf[0] ^ rx_buf[1] ^ rx_buf[2] ^ rx_buf[3];
	if (bcc != rx_buf[4])
		return RC522_ERR_CRC;

	for (i = 0U; i < 4U; i++) 
		uid_bytes[i] = rx_buf[i];

	tx_buf[0] = sel_cmd;
	tx_buf[1] = 0x70U;
	tx_buf[2] = rx_buf[0];
	tx_buf[3] = rx_buf[1];
	tx_buf[4] = rx_buf[2];
	tx_buf[5] = rx_buf[3];
	tx_buf[6] = rx_buf[4];

	status = rc522_calc_crc(h, tx_buf, 7U, &crc_h, &crc_l);
	if (status != RC522_OK) 
		return status;

	tx_buf[7] = crc_l;
	tx_buf[8] = crc_h;

	rx_len = sizeof(rx_buf);
	status = rc522_transceive(h, RC522_CMD_TRANSCEIVE,
				tx_buf, 9U,
				rx_buf, &rx_len,
				&last_bits, 0U);

	if (status != RC522_OK)
		return status;
	if (rx_len  < 1U)
		return RC522_ERR_NOCARD;

	*sak_out = rx_buf[0];
	return RC522_OK;
}

rc522_status_t rc522_read_uid(const rc522_handle_t *h, rc522_uid_t *uid)
{
	uint8_t		sak = 0U;
	uint8_t		uid_buf[4];
	uint8_t		cascade;
	rc522_status_t status;

	if (uid == NULL)
		return RC522_ERR_PARAM;

	memset(uid, 0, sizeof(rc522_uid_t));

	for (cascade = 0U; cascade < 3U; cascade++)
	{
		uint8_t sel_cmd;
		uint8_t start;

		if	  (cascade == 0U)
			sel_cmd = PICC_CMD_SEL_CL1;
		else if (cascade == 1U) 
			sel_cmd = PICC_CMD_SEL_CL2;
		else
			sel_cmd = PICC_CMD_SEL_CL3;

		status = rc522_anticoll_select(h, sel_cmd, uid_buf, &sak);
		if (status != RC522_OK)
			return status;

		start = uid->size;

		/*
		 * CT (0x88): cascade marker — not included in the final UID.
		 * Its presence indicates the UID continues at the next level.
		 */
		if (uid_buf[0] == PICC_CMD_CT)
		{
			uid->bytes[start + 0U] = uid_buf[1];
			uid->bytes[start + 1U] = uid_buf[2];
			uid->bytes[start + 2U] = uid_buf[3];
			uid->size += 3U;
		} 
		else
		{
			uid->bytes[start + 0U] = uid_buf[0];
			uid->bytes[start + 1U] = uid_buf[1];
			uid->bytes[start + 2U] = uid_buf[2];
			uid->bytes[start + 3U] = uid_buf[3];
			uid->size += 4U;
		}

		uid->sak = sak;

		if ((sak & 0x04U) == 0U)
			break;
	}

	return RC522_OK;
}