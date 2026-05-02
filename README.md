# RC522 SPI Driver

Bare-metal SPI driver for the MFRC522 RFID reader, written in C for embedded systems.
No third-party RFID library — all communication is implemented from scratch against the RC522 datasheet.

Targets the ESP32 (Espressif / Arduino framework) using the hardware VSPI bus.

## Features

- ISO 14443A card detection via REQA (7-bit short frame)
- Anticollision and Select sequence with BCC integrity check
- Hardware CRC_A computation via the RC522 onboard coprocessor
- Multi-level cascade UID support (4 / 7 / 10 bytes)
- MISRA-C aligned: explicit integer types, no implicit conversions, `static` scoping

## Hardware

| RC522 Pin | ESP32 GPIO |
|-----------|-----------|
| SDA (CS)  | 5         |
| SCK       | 18        |
| MOSI      | 23        |
| MISO      | 19        |
| RST       | 27        |
| 3.3V      | 3.3V      |
| GND       | GND       |

> The RC522 operates at **3.3V only**. IRQ pin is not used.

## References

- [MFRC522 Datasheet](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf) — NXP Semiconductors
- ISO/IEC 14443-3 — Identification cards, proximity cards, anticollision and initialisation