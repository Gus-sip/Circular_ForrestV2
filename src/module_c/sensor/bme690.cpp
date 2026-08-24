#include <Wire.h>
#include <7Semi_BME690.h>

#define I2C_SDA 8
#define I2C_SCL 16
#define BME690_I2C_ADDR 0x76 // corregido: el scanner encontro el sensor aqui

BME69X_7Semi sensor;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=== TEST BME690 (libreria 7Semi) ===");

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Escaneando bus I2C...");
  byte encontrados = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Dispositivo I2C encontrado en 0x");
      Serial.println(addr, HEX);
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Serial.println("  Ningun dispositivo I2C detectado. Revisa cableado/alimentacion.");
  }

  if (!sensor.begin(BME690_I2C_ADDR)) {
    Serial.println("ERROR: No se pudo inicializar el BME690.");
    while (1) { delay(1000); }
  }

  Serial.println("BME690 encontrado!");
}

void loop() {
  float temperature, humidity, pressure, gas;

  if (sensor.getData(temperature, humidity, pressure, gas)) {
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print(" C  |  Humedad: ");
    Serial.print(humidity);
    Serial.print(" %  |  Presion: ");
    Serial.print(pressure);
    Serial.print(" hPa  |  Gas: ");
    Serial.print(gas);
    Serial.println(" ohm");
  } else {
    Serial.println("Error leyendo el sensor");
  }

  delay(2000);
}
