// PRUEBA AISLADA DEL BUS I2C. Nada del firmware de produccion entra aqui.
//
// Para que existe: el 2026-10-08 los tres sensores de I2C de C3 dejaron de
// responder y habia que decidir si la causa era el firmware nuevo o el hardware.
// Discutirlo no sirve; esto lo zanja. Si los sensores no aparecen ni en un
// programa que solo enciende los rails y escanea el bus, no hay codigo nuestro al
// que culpar, porque no hay codigo nuestro.
//
// Hace lo minimo imprescindible y en el orden mas favorable posible:
//   1. enciende los dos rails de sensores, con la polaridad correcta
//   2. espera a que arranquen
//   3. mide el estado electrico de SDA y SCL
//   4. escanea el bus entero
//   5. interroga al BME690 en sus dos direcciones posibles
// y lo repite en bucle para que se pueda conectar y desconectar en caliente.
#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>

#include "pins.h"

// Igual que i2cLineState() del firmware: distingue una linea LIBRE de una
// FLOTANTE (sin pull-up) y de una SUJETA a masa. El nivel por si solo no las
// separa, y confundirlas fue lo que mando el diagnostico por mal camino.
static const char *estadoLinea(uint8_t pin) {
  pinMode(pin, INPUT);
  delayMicroseconds(200);
  if (digitalRead(pin) == HIGH) return "LIBRE";

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(50);
  pinMode(pin, INPUT);
  delayMicroseconds(5);
  const int alSoltar = digitalRead(pin);
  delayMicroseconds(200);
  const int despues = digitalRead(pin);

  if (alSoltar == LOW) return "SUJETA a masa";
  if (despues == LOW) return "FLOTANTE (sin pull-up)";
  return "LIBRE (lenta)";
}

static void escanear() {
  int encontrados = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    ACK en 0x%02X\n", addr);
      encontrados++;
    }
  }
  if (encontrados == 0) Serial.println("    (nadie responde en todo el bus)");
}

// El chip ID del BME690 debe valer 0x61. Se pregunta con dos transacciones
// SEPARADAS, cada una con su STOP, en vez de con repeated start: endTransmission
// (false) no ejecuta nada en ESP32 y devuelve 0 aunque no haya nadie, que es
// precisamente el falso positivo que costo una hora de diagnostico.
static void preguntarBme(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  const uint8_t wr = Wire.endTransmission();  // con STOP: esta SI se ejecuta
  if (wr != 0) {
    Serial.printf("    0x%02X no acusa la escritura (endTransmission=%u)\n", addr,
                  (unsigned)wr);
    return;
  }
  if (Wire.requestFrom((int)addr, 1) != 1) {
    Serial.printf("    0x%02X acuso la escritura pero no devolvio dato\n", addr);
    return;
  }
  const uint8_t id = (uint8_t)Wire.read();
  Serial.printf("    0x%02X chip ID = 0x%02X %s\n", addr, id,
                id == 0x61 ? "<-- BME690 VIVO" : "(se esperaba 0x61)");
}

void setup() {
  Serial.begin(115200);
  delay(2500);  // que el puerto USB enumere antes de la primera linea
  Serial.println("\n\n=== PRUEBA AISLADA DEL BUS I2C (sin firmware de produccion) ===");

  Serial.printf("SDA=GPIO%d  SCL=GPIO%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
  Serial.printf("Antes de nada:             SDA=%s  SCL=%s\n", estadoLinea(PIN_I2C_SDA),
                estadoLinea(PIN_I2C_SCL));

  // SOLTAR EL ENCLAVADO DE PAD, que es la unica via por la que el SOFTWARE puede
  // dejar un pin pegado a masa de forma que sobreviva a cambiar el firmware.
  //
  // gpio_hold_en() congela el nivel de un pad, y ese enclavado persiste a traves
  // del reset Y DEL REFLASHEO: lo mantiene el dominio RTC, no el programa. Si
  // alguna version anterior dejo GPIO2 enclavado como salida en bajo, todo
  // firmware posterior se encuentra SCL a masa y no hay forma de levantarlo -
  // ni escribiendo el pin, porque el enclavado ignora al periferico.
  //
  // El firmware de produccion solo suelta GPIO10/11/13 (kHeldGates), asi que GPIO2
  // nunca se libera. Esta es la hipotesis que esto prueba.
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)PIN_I2C_SCL);
  gpio_hold_dis((gpio_num_t)PIN_I2C_SDA);
  delay(5);
  Serial.printf("Tras soltar el enclavado:  SDA=%s  SCL=%s\n", estadoLinea(PIN_I2C_SDA),
                estadoLinea(PIN_I2C_SCL));

  // Rails ON con su polaridad real: EN_A (3V3) es activo-bajo y EN_B (5V)
  // activo-alto. Invertir uno fue un error real de este proyecto.
  pinMode(PIN_PCB_EN_A, OUTPUT);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  Serial.println("Rails 3V3 y 5V encendidos, esperando 1s...");
  delay(1000);

  Serial.printf("Con los rails ON:          SDA=%s  SCL=%s\n", estadoLinea(PIN_I2C_SDA),
                estadoLinea(PIN_I2C_SCL));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Serial.println("Wire iniciado a 100 kHz.");
}

// El BMV080 acusa su direccion en el escaneo pero el SDK de Bosch falla al
// abrirlo con E_BMV080_ERROR_HW_WRITE (106), que significa que LA ESCRITURA
// falla, no que no encuentre el chip.
//
// Esto separa las dos capas: si una escritura I2C cruda al mismo chip SI es
// aceptada, el problema esta en el SDK o en la libreria de bus que hay debajo; si
// tampoco pasa, el problema es el chip o su habilitacion.
static void probarBmv(uint8_t addr) {
  // Escritura de longitud cero: solo la direccion. Es la que el escaneo ya hace.
  Wire.beginTransmission(addr);
  const uint8_t soloDir = Wire.endTransmission();

  // Escritura de un registro de 16 bits MSB primero, que es como direcciona este
  // chip. Si el chip esta vivo y habilitado deberia acusarla.
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  const uint8_t conReg = Wire.endTransmission();

  Serial.printf("    0x%02X  solo-direccion=%u  registro-16bit=%u  -> %s\n", addr,
                (unsigned)soloDir, (unsigned)conReg,
                (soloDir == 0 && conReg == 0)
                    ? "acepta escrituras crudas: el chip responde, sospechoso el SDK"
                    : (soloDir == 0 ? "acusa la direccion pero RECHAZA el registro"
                                    : "no responde"));
}

void loop() {
  Serial.println("\n--- escaneo ---");
  escanear();
  Serial.println("--- BME690 por chip ID ---");
  preguntarBme(0x76);
  preguntarBme(0x77);
  Serial.println("--- BMV080 crudo, sin SDK ---");
  for (uint8_t a : {0x57, 0x56, 0x55, 0x54}) probarBmv(a);
  Serial.printf("--- lineas: SDA=%s  SCL=%s ---\n", estadoLinea(PIN_I2C_SDA),
                estadoLinea(PIN_I2C_SCL));
  // estadoLinea() deja los pines como GPIO sueltos, asi que hay que devolverlos
  // al periferico I2C antes del siguiente escaneo.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Serial.flush();
  delay(4000);
}
