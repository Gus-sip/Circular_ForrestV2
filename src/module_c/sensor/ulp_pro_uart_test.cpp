/*
 * Calypso ULP PRO Wind Sensor - UART Test Sketch
 * Target: ESP32-S3 Super Mini (PlatformIO, Arduino framework)
 *
 * Wiring (UART/I2C version, CMI1032):
 *   Brown  -> 3V3            (VCC, sensor accepts 3.3-18 VDC)
 *   White  -> GND
 *   Green  -> GPIO 5 (RX)    (sensor TX)
 *   Yellow -> GPIO 6 (TX)    (sensor RX - only needed for poll mode)
 *
 * Sensor defaults: 38400 baud, 8N1, streaming NMEA 0183:
 *   $IIMWV,<angle>,R,<speed>,N,A*<checksum>
 */

#include <Arduino.h>

#define WIND_RX_PIN 5   // connect to sensor GREEN wire (TX out of sensor)
#define WIND_TX_PIN 6   // connect to sensor YELLOW wire (RX into sensor)
#define WIND_BAUD   38400

HardwareSerial WindSerial(1);   // UART1

String nmeaBuffer = "";

// Validate NMEA checksum: XOR of chars between '$' and '*'
bool checksumOk(const String &sentence) {
  int star = sentence.indexOf('*');
  if (star < 0 || sentence.length() < star + 3) return false;

  uint8_t calc = 0;
  for (int i = 1; i < star; i++) calc ^= sentence[i];

  uint8_t received = (uint8_t) strtol(sentence.substring(star + 1, star + 3).c_str(), nullptr, 16);
  return calc == received;
}

// Parse $--MWV,angle,R,speed,units,status*hh
void parseMWV(const String &sentence) {
  // Split into fields
  String fields[6];
  int fieldIdx = 0, start = 0;
  for (int i = 0; i < (int)sentence.length() && fieldIdx < 6; i++) {
    char c = sentence[i];
    if (c == ',' || c == '*') {
      fields[fieldIdx++] = sentence.substring(start, i);
      start = i + 1;
    }
  }

  if (fieldIdx < 6) {
    Serial.println("  [parse error: not enough fields]");
    return;
  }

  float angle = fields[1].toFloat();
  float speed = fields[3].toFloat();
  String units = fields[4];   // K = km/h, M = m/s, N = knots
  String status = fields[5];  // A = valid

  String unitName = (units == "M") ? "m/s" : (units == "N") ? "knots" : (units == "K") ? "km/h" : units;

  Serial.printf("  Wind: %.1f deg  |  %.1f %s  |  status: %s\n",
                angle, speed, unitName.c_str(),
                status == "A" ? "VALID" : "INVALID");
}

void setup() {
  Serial.begin(115200);         // USB serial monitor
  delay(2000);                  // give USB CDC time to enumerate

  WindSerial.begin(WIND_BAUD, SERIAL_8N1, WIND_RX_PIN, WIND_TX_PIN);

  Serial.println("=== Calypso ULP PRO UART test ===");
  Serial.printf("Listening on UART1 @ %d baud (RX=%d, TX=%d)\n", WIND_BAUD, WIND_RX_PIN, WIND_TX_PIN);
  Serial.println("Waiting for NMEA sentences...\n");
}

void loop() {
  while (WindSerial.available()) {
    char c = WindSerial.read();

    if (c == '\n') {
      nmeaBuffer.trim();
      if (nmeaBuffer.length() > 0) {
        // Show the raw sentence exactly as received
        Serial.print("RAW: ");
        Serial.println(nmeaBuffer);

        if (nmeaBuffer.startsWith("$") && nmeaBuffer.indexOf("MWV") == 3) {
          if (checksumOk(nmeaBuffer)) {
            parseMWV(nmeaBuffer);
          } else {
            Serial.println("  [checksum FAILED - check wiring/baud]");
          }
        }
      }
      nmeaBuffer = "";
    } else if (c != '\r') {
      nmeaBuffer += c;
      if (nmeaBuffer.length() > 120) nmeaBuffer = "";  // guard against garbage
    }
  }
}
