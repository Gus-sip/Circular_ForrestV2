#include <Arduino.h>

// Standalone UART test for the Cubic CM1106SL-NS CO2 sensor.
// Wiring: sensor RXD -> GPIO43 (board TX), sensor TXD -> GPIO44 (board RX)
//         Sensor EN -> GPIO4 (driven HIGH to power on; active-high assumed)
// Uses UART0 (Serial0) since ARDUINO_USB_CDC_ON_BOOT routes the USB "Serial" console over native USB,
// freeing the board's silkscreened TX/RX pins for this sensor.

#define CO2_BAUD 9600
#define EN_PIN 4

#define co2Serial Serial0

static const uint8_t READ_CO2_CMD[] = {0x11, 0x01, 0x01, 0xED};

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  co2Serial.begin(CO2_BAUD, SERIAL_8N1, RX, TX);

  delay(3000);
  Serial.println("starting");
}

void loop() {
  static uint32_t lastRequest = 0;

  if (millis() - lastRequest < 3000) return;
  lastRequest = millis();

  while (co2Serial.available()) co2Serial.read();

  co2Serial.write(READ_CO2_CMD, sizeof(READ_CO2_CMD));

  uint8_t resp[16];
  uint8_t len = 0;
  uint32_t start = millis();
  while (millis() - start < 500 && len < sizeof(resp)) {
    if (co2Serial.available()) {
      resp[len++] = co2Serial.read();
    }
  }

  if (len == 0) {
    Serial.println("TIMEOUT - no response");
    return;
  }

  Serial.print("RX:");
  for (uint8_t i = 0; i < len; i++) {
    Serial.printf(" %02X", resp[i]);
  }
  Serial.println();

  if (len >= 8 && resp[0] == 0x16 && resp[1] == 0x05 && resp[2] == 0x01) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 7; i++) sum += resp[i];
    uint8_t checksum = (uint8_t)(256 - (sum % 256));
    if (checksum == resp[7]) {
      uint16_t ppm = resp[3] * 256 + resp[4];
      Serial.printf("CO2: %u ppm\n", ppm);
    }
  }
}
