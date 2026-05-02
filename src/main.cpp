#include "rc522.hpp"
#include <Arduino.h>

static rc522_handle_t g_rfid;

void setup(void)
{
	Serial.begin(115200);

	rc522_init(&g_rfid, RC522_PIN_SDA, RC522_PIN_RST);

	uint8_t ver = rc522_get_version(&g_rfid);
	Serial.printf("[RC522] Version: 0x%02X (attendu 0x91 ou 0x92)\n", ver);

	if (ver != 0x91U && ver != 0x92U)
		Serial.println("[RC522] ERREUR : puce non détectée — vérifier câblage SPI");
	else
		Serial.println("[RC522] Init OK — approcher une carte RFID");

}

void loop(void)
{
	static uint32_t last_ms = 0U;

	if ((millis() - last_ms) < 200U) 
		return;
	last_ms = millis();

	if (rc522_is_card_present(&g_rfid) != RC522_OK) 
		return;

	rc522_uid_t uid;
	rc522_status_t st = rc522_read_uid(&g_rfid, &uid);

	if (st == RC522_OK)
	{
		Serial.printf("[RC522] Carte détectée — UID (%d octets): ", uid.size);
		for (uint8_t i = 0U; i < uid.size; i++)
			Serial.printf("%02X ", uid.bytes[i]);
		Serial.printf("| SAK: 0x%02X\n", uid.sak);
	} 
	else
		Serial.printf("[RC522] Erreur lecture UID: %d\n", st);
}