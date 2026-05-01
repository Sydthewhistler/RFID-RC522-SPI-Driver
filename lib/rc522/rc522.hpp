/**
 * @file rc522.hpp
 * @brief SPI RFID RC522 driver — MISRA-C embedded style
 *
 * SPI Mode 0 (CPOL=0, CPHA=0), max 10 MHz.
 * Frame format: address byte [RW|ADDR<<1] followed by the data byte.
 */

#ifndef RC522_HPP
#define RC522_HPP

#include <stdint.h>
#include <stdbool.h>

/* ─── ESP32 Pins ────────────────────────────────────────────────── */
#define RC522_PIN_SDA		5U
#define RC522_PIN_SCK		18U
#define RC522_PIN_MOSI		23U
#define RC522_PIN_MISO		19U
#define RC522_PIN_RST		27U

/* ─── Registers (datasheet §9.2) ───────────────────────────────── */
#define RC522_REG_COMMAND		0x01U
#define RC522_REG_COM_IEN		0x02U
#define RC522_REG_DIV_IEN		0x03U
#define RC522_REG_COM_IRQ		0x04U
#define RC522_REG_DIV_IRQ		0x05U
#define RC522_REG_ERROR			0x06U
#define RC522_REG_STATUS1		0x07U
#define RC522_REG_STATUS2		0x08U
#define RC522_REG_FIFO_DATA		0x09U
#define RC522_REG_FIFO_LEVEL	0x0AU
#define RC522_REG_WATER_LEVEL	0x0BU
#define RC522_REG_CONTROL		0x0CU
#define RC522_REG_BIT_FRAMING	0x0DU
#define RC522_REG_COLL			0x0EU
#define RC522_REG_MODE			0x11U
#define RC522_REG_TX_MODE		0x12U
#define RC522_REG_RX_MODE		0x13U
#define RC522_REG_TX_CONTROL	0x14U
#define RC522_REG_TX_ASK		0x15U
#define RC522_REG_TX_SEL		0x16U
#define RC522_REG_RX_SEL		0x17U
#define RC522_REG_RX_THRESHOLD	0x18U
#define RC522_REG_DEMOD			0x19U
#define RC522_REG_MF_TX			0x1CU
#define RC522_REG_MF_RX			0x1DU
#define RC522_REG_SERIAL_SPEED	0x1FU
#define RC522_REG_CRC_RESULT_H	0x21U
#define RC522_REG_CRC_RESULT_L	0x22U
#define RC522_REG_MOD_WIDTH		0x24U
#define RC522_REG_RF_CFG		0x26U
#define RC522_REG_GS_N			0x27U
#define RC522_REG_CW_GS_P		0x28U
#define RC522_REG_MOD_GS_P		0x29U
#define RC522_REG_T_MODE		0x2AU
#define RC522_REG_T_PRESCALER	0x2BU
#define RC522_REG_T_RELOAD_H	0x2CU
#define RC522_REG_T_RELOAD_L	0x2DU
#define RC522_REG_T_COUNTER_H	0x2EU
#define RC522_REG_T_COUNTER_L	0x2FU
#define RC522_REG_AUTO_TEST		0x36U
#define RC522_REG_VERSION		0x37U  /**< 0x91 = v1, 0x92 = v2 */

/* ─── Internal RC522 commands ──────────────────────────────────── */
#define RC522_CMD_IDLE			0x00U
#define RC522_CMD_CALC_CRC		0x03U
#define RC522_CMD_TRANSMIT		0x04U
#define RC522_CMD_RECEIVE		0x08U
#define RC522_CMD_TRANSCEIVE	0x0CU  /**< TX then RX, used for ISO14443A */
#define RC522_CMD_MF_AUTHENT	0x0EU
#define RC522_CMD_SOFT_RESET	0x0FU

/* ─── PICC ISO 14443A commands ─────────────────────────────────── */
#define PICC_CMD_REQA			0x26U  /**< Detects cards in the field (7 bits) */
#define PICC_CMD_WUPA			0x52U
#define PICC_CMD_CT				0x88U  /**< Cascade tag: incomplete UID, next level */
#define PICC_CMD_SEL_CL1		0x93U
#define PICC_CMD_SEL_CL2		0x95U
#define PICC_CMD_SEL_CL3		0x97U
#define PICC_CMD_HLTA			0x50U
#define PICC_CMD_READ			0x30U
#define PICC_CMD_WRITE			0xA0U

/* ─── SPI Configuration ─────────────────────────────────────────── */
#define RC522_SPI_FREQ_HZ		5000000UL

/* ─── Return codes ───────────────────────────────────────────── */
typedef enum {
	RC522_OK		=0,
	RC522_ERR_TIMEOUT	= 1,
	RC522_ERR_CRC		= 2,
	RC522_ERR_COLLISION	= 3,
	RC522_ERR_NOCARD	= 4,
	RC522_ERR_PARAM		= 5,
} rc522_status_t;

/* ─── ISO 14443A card UID ──────────────────────────────────────── */
#define RC522_UID_MAX_BYTES 10U  /**< 4 / 7 / 10 bytes depending on card type */

typedef struct {
	uint8_t bytes[RC522_UID_MAX_BYTES];
	uint8_t size;  /**< Number of valid bytes */
	uint8_t sak;   /**< Select Acknowledge — identifies the card type */
} rc522_uid_t;

/* ─── Driver handle ─────────────────────────────────────────────── */
typedef struct {
	uint8_t pin_cs;
	uint8_t pin_rst;
	bool	initialized;
} rc522_handle_t;

/* ─── Public API ──────────────────────────────────────────────── */

/**
 * @brief Initializes the SPI bus and configures the RC522.
 * @param h	   Handle to initialize
 * @param pin_cs  Chip select GPIO
 * @param pin_rst Reset GPIO (active low)
 */
void rc522_init(rc522_handle_t *h, uint8_t pin_cs, uint8_t pin_rst);

/** @brief Reads an 8-bit register. */
uint8_t rc522_read_reg(const rc522_handle_t *h, uint8_t reg);

/** @brief Writes an 8-bit register. */
void rc522_write_reg(const rc522_handle_t *h, uint8_t reg, uint8_t val);

/** @brief Sets bits defined by mask in a register. */
void rc522_set_bits(const rc522_handle_t *h, uint8_t reg, uint8_t mask);

/** @brief Clears bits defined by mask in a register. */
void rc522_clear_bits(const rc522_handle_t *h, uint8_t reg, uint8_t mask);

/**
 * @brief Detects the presence of a card using REQA (ISO 14443A).
 * @return RC522_OK if a card responds, RC522_ERR_NOCARD otherwise.
 */
rc522_status_t rc522_is_card_present(const rc522_handle_t *h);

/**
 * @brief Reads the UID of the present card (supports 4/7/10 byte cascade).
 * @param uid  Structure to fill
 * @return	 RC522_OK on success
 */
rc522_status_t rc522_read_uid(const rc522_handle_t *h, rc522_uid_t *uid);

/** @brief Returns the hardware version read from REG_VERSION. */
uint8_t rc522_get_version(const rc522_handle_t *h);

#endif /* RC522_HPP */