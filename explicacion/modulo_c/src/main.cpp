/*
 * CHIP FOREST - MODULO C (nodo sensor)  -  VERSION SIMPLIFICADA PARA EXPLICAR
 *
 * Que hace este nodo, en una frase:
 *   duerme casi todo el tiempo y se despierta SOLO cuando a algun sensor le toca
 *   medir o cuando toca enviar; cada sensor tiene su propio periodo, que depende
 *   del estado de alarma, y todo va alineado a la hora real que le pasa Modulo B.
 *
 * Las tres piezas, en el orden en que aparecen en el archivo:
 *
 *   A. RELOJ       el nodo no tiene reloj. Cuenta el tiempo el mismo y lo corrige
 *                  con la hora de red que Modulo B le manda (CFG,TIME).
 *   B. CALENDARIO  una tabla de trabajos (un sensor = un trabajo, mas el envio),
 *                  cada uno con su periodo en minutos. Se despierta para el
 *                  proximo que vence, con la antelacion que necesita ese sensor.
 *   C. ALARMA      NORMAL / PRE-ALARMA / ALARMA. En los estados altos los periodos
 *                  se acortan y el nodo deja de dormir.
 *
 * Cada despertar de sueno profundo es un arranque nuevo: se ejecuta setup() desde
 * el principio. Todo lo que tiene que sobrevivir al sueno (calendario, reloj,
 * lecturas guardadas, estado de alarma) vive en memoria RTC (RTC_DATA_ATTR).
 *
 * Diferencias con el firmware real (src/module_c/communications/chip_forest_lora_tx.cpp):
 *   no estan el modo seguro, los comandos CFG distintos de TIME, el LED, la medida
 *   de bateria, ni las cinco lecturas limpias que exige el real para bajar la alarma.
 *   El calendario, los periodos, los tiempos y el reloj son los del firmware real.
 */

#include <Arduino.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "pins.h"     // que pin va a que
#include "Config.h"   // direcciones I2C y velocidades de los sensores
#include "RYLR998.h"  // driver de la radio LoRa
#include "Bme690Sensor.h"
#include "Bmv080Sensor.h"
#include "Cm1106Sensor.h"
#include "Sen0466Sensor.h"
#include "CalypsoSensor.h"

// ===========================================================================
// 1. CONFIGURACION
// ===========================================================================

// Radio LoRa. Estos valores TIENEN que ser iguales en Modulo B o no se oyen.
#define LORA_MY_ADDR       4            // direccion de este nodo (C1=1, C2=3, C3=4)
#define LORA_RX_ADDR       2            // direccion de Modulo B
#define LORA_NETWORK_ID    5
#define LORA_BAND_HZ       868000000UL  // banda europea EU868
#define LORA_TX_POWER_DBM  14           // menos de 22 dBm: el pico de corriente es menor
#define LORA_RX_PIN        4
#define LORA_TX_PIN        5
#define LORA_LISTEN_MS     2000         // ventana de escucha tras transmitir

// Tiempos de los sensores, MEDIDOS en el banco (nodo C3, 2026-10-06 y 2026-10-09).
#define BMV080_PREHEAT_MS  2000         // el laser necesita calentarse antes de medir
#define BMV080_WINDOW_MS   8000         // tiempo maximo esperando una medida de PM
#define CALYPSO_BOOT_MS    20000UL      // sin trama a los 12,4 s de encenderlo; con trama a los 22,7 s
#define CALYPSO_WINDOW_MS  1500         // tiempo escuchando su trama
#define SLOT_GAP_MS        1500         // reposo entre un sensor y el siguiente

// Despues de encender o resetear (no al despertar del sueno), el nodo se queda
// despierto este tiempo: dormido, el puerto USB desaparece y no se puede reflashear.
#define COLD_BOOT_AWAKE_MS 30000UL

// En pre-alarma y alarma el nodo no duerme: mira el calendario cada 2 s.
#define ALARM_LOOP_MS      2000

// La pre-alarma se confirma: si dura 60 minutos, pasa a alarma.
#define PREALARM_CONFIRM_MS 3600000ULL

// "Sin medir". Un sensor que no toca, o que falla, no manda un numero: manda esto,
// y Modulo B no publica ese campo. Asi "no se midio" no se confunde con "midio 0".
// -999 y no -1 porque la temperatura puede valer -1 grado de verdad.
#define SIN_DATO -999.0f

// ===========================================================================
// 2. OBJETOS: un objeto por sensor y uno para la radio
// ===========================================================================

Bme690Sensor  bme690(BME690_I2C_ADDR);    // temperatura, humedad, presion, gas (VOC)
Sen0466Sensor sen0466(SEN0466_I2C_ADDR);  // monoxido de carbono (CO)
Bmv080Sensor  bmv080(BMV080_I2C_ADDR);    // particulas PM1 / PM2.5 / PM10
Cm1106Sensor  cm1106(Serial2, PIN_CM1106_RX, PIN_CM1106_TX, PIN_CM1106_EN,
                     CM1106_WARMUP_MS, CM1106_BAUD);  // CO2
CalypsoSensor calypso(Serial1, PIN_CALYPSO_RX, PIN_CALYPSO_TX, CALYPSO_BAUD);  // viento
RYLR998       radio(Serial0, LORA_RX_PIN, LORA_TX_PIN);

enum Estado : uint8_t { NORMAL = 0, PREALARMA = 1, ALARMA = 2 };

// ===========================================================================
// 3. MEMORIA QUE SOBREVIVE AL SUENO (RTC)
// ===========================================================================
//
// RTC_DATA_ATTR guarda la variable en la memoria RTC, que sigue alimentada durante
// el sueno profundo. Una variable normal volveria a su valor inicial en cada
// despertar, y el nodo olvidaria la hora, el calendario y las lecturas.
// Se ponen a cero solo en un arranque en frio - ver iniciarMemoriaRTC().

// --- Reloj ---
RTC_DATA_ATTR uint64_t nodoBaseMs;        // tiempo del nodo acumulado en despertares anteriores
RTC_DATA_ATTR bool     relojSincronizado; // ¿ha recibido alguna vez la hora de Modulo B?
RTC_DATA_ATTR uint64_t epocaMsEnSync;     // hora real recibida en la ultima sincronizacion
RTC_DATA_ATTR uint64_t nodoMsEnSync;      // tiempo del nodo en ese mismo instante
RTC_DATA_ATTR float    deriva;            // ms reales por cada ms que cuenta el nodo

// --- Calendario ---
#define NUM_TRABAJOS 7
RTC_DATA_ATTR uint64_t proximoVencimientoMs[NUM_TRABAJOS];

// --- Alarma ---
RTC_DATA_ATTR uint8_t  estado;
RTC_DATA_ATTR uint64_t preAlarmaDesdeMs;  // cuando entro en pre-alarma (tiempo del nodo)

// Ultimo valor conocido de cada magnitud. Es lo que evalua la alarma: un incendio
// detectado por las particulas no se "olvida" porque el siguiente despertar solo
// mida la temperatura.
struct Valores {
  float temp, hum, pres, gas;
  float pm1, pm25, pm10;
  float co2, co, coTemp;
  float windAngle, windSpeed;
  bool windValid;
};
RTC_DATA_ATTR Valores ultimo;

// --- Lecturas guardadas hasta el proximo envio ---
// Cada fila lleva SOLO lo que se midio en ese despertar (el resto a SIN_DATO) y
// cuando se tomo, para que Modulo B la feche bien aunque llegue media hora despues.
struct Fila {
  Valores v;
  uint32_t tomadaS;  // segundo del nodo en que se tomo
};
#define FILAS_MAX 8
RTC_DATA_ATTR Fila almacen[FILAS_MAX];
RTC_DATA_ATTR uint8_t almacenN;

// ===========================================================================
// A. EL RELOJ
// ===========================================================================
//
// El ESP32 no tiene reloj de tiempo real con pila. Mientras duerme cuenta el
// tiempo con un oscilador RC interno, que se desvia un 1-2 %. Y millis() vuelve a
// cero en cada despertar.
//
// Por eso el nodo lleva DOS tiempos:
//
//   tiempo del nodo  = lo que el nodo cree que ha pasado desde que se encendio.
//                      Siempre existe. Se acumula en dormir(): lo que estuvo
//                      despierto + lo que va a dormir.
//
//   hora real        = la hora de la red movil, que Modulo B saca del modem y le
//                      manda al nodo con "CFG,TIME=<segundos desde 1970>".
//                      A partir de ahi:  hora real = hora recibida + tiempo
//                      transcurrido desde entonces x deriva.
//
// La DERIVA se aprende sola: en cada sincronizacion se compara cuanto tiempo cree
// el nodo que ha pasado contra cuanto dice la red. Si el nodo conto 1000 s y la red
// dice 1013, la deriva es 1,013 (su oscilador va un 1,3 % lento).

#define DERIVA_MIN 0.97f              // un RC se desvia un 1-2 %, no un 10 %:
#define DERIVA_MAX 1.03f              // fuera de esto, la causa es otra y no se aplica
#define DERIVA_TRAMO_MIN_MS 300000ULL // por debajo de 5 min, medir un 1 % es medir ruido

uint64_t tiempoNodoMs() { return nodoBaseMs + (uint64_t)millis(); }

// Hora real en ms desde 1970, o 0 si todavia no se ha sincronizado nunca.
uint64_t horaRealMs() {
  if (!relojSincronizado) return 0;
  uint64_t transcurrido = tiempoNodoMs() - nodoMsEnSync;
  return epocaMsEnSync + (uint64_t)((double)transcurrido * (double)deriva);
}

// El calendario se planifica sobre la hora real si la hay, y si no sobre el tiempo
// del nodo. La formula es la misma; solo cambia el origen.
uint64_t ahoraMs() { return relojSincronizado ? horaRealMs() : tiempoNodoMs(); }

void replanificar(const char *motivo);  // definida en la parte B

// Se llama cuando llega "CFG,TIME=<s>" en la ventana de escucha.
void sincronizarReloj(uint32_t epocaS) {
  if (epocaS < 1672531200UL) return;  // antes de 2023: el modem aun no tiene hora

  const uint64_t epocaMs = (uint64_t)epocaS * 1000ULL;
  const uint64_t nodoAhora = tiempoNodoMs();

  // Aprender la deriva. Hace falta una sincronizacion anterior para comparar.
  if (relojSincronizado && epocaMs > epocaMsEnSync && nodoAhora > nodoMsEnSync) {
    const uint64_t real = epocaMs - epocaMsEnSync;   // lo que paso de verdad
    const uint64_t nodo = nodoAhora - nodoMsEnSync;  // lo que conto el nodo
    if (nodo >= DERIVA_TRAMO_MIN_MS) {
      float f = (float)((double)real / (double)nodo);
      if (f >= DERIVA_MIN && f <= DERIVA_MAX) {
        deriva = f;
      } else {
        Serial.printf("[reloj] factor %.4f fuera de rango - descartado\n", f);
      }
    }
  }

  const bool primeraVez = !relojSincronizado;
  epocaMsEnSync = epocaMs;
  nodoMsEnSync = nodoAhora;
  relojSincronizado = true;
  Serial.printf("[reloj] sincronizado: %lu s, deriva %+ld ppm\n", (unsigned long)epocaS,
                (long)((deriva - 1.0f) * 1000000.0f));

  // La primera vez cambia la base de tiempo del calendario (de "segundos desde que
  // se encendio" a "milisegundos desde 1970"), asi que hay que rehacerlo. Ademas es
  // lo que ALINEA a los nodos: desde ahora todos miden en los mismos minutos.
  if (primeraVez) replanificar("primera hora de red");
}

// ===========================================================================
// B. EL CALENDARIO
// ===========================================================================
//
// Un trabajo por sensor, mas el envio. El BME690 tiene DOS: la lectura rapida de
// temperatura/humedad/presion (sin calefactor) y la de gas (calefactor a 320 C).
//
// PERIODO: en minutos, uno por estado de alarma. En alarma todo va mas rapido.
//
// PREPARACION: cuanto ANTES de su minuto hay que despertar para que el dato exista
// EN ese minuto. Es el tiempo que tarda el sensor en estar listo:
//   Calypso: 20 s de arranque + 1,5 s de escucha. BMV080: 5 s de arranque + 2 s de
//   laser. Gas del BME690: ~11 s de calefactor.
//
// DESFASE: solo el envio. En el minuto 30 vencen cinco lecturas a la vez y van en
// fila (~19 s medidos), asi que se transmite un minuto despues, con todo ya leido.
// No es un periodo de 31 min: es periodo 30 + desfase 1, siempre en la misma rejilla.

enum Trabajo : uint8_t {
  T_BME_THP = 0,  // temperatura, humedad, presion
  T_BME_VOC,      // resistencia de gas (compuestos organicos volatiles)
  T_CM1106,       // CO2
  T_SEN0466,      // CO
  T_BMV080,       // particulas
  T_CALYPSO,      // viento
  T_ENVIO,        // transmitir a Modulo B
};

struct DefTrabajo {
  const char *nombre;
  uint16_t periodoMin[3];  // NORMAL, PREALARMA, ALARMA
  uint32_t preparacionMs;
  uint32_t desfaseMs;      // solo en NORMAL
};

const DefTrabajo TRABAJOS[NUM_TRABAJOS] = {
  //  nombre          NORM PREA ALARM  preparacion                          desfase
  {"BME690 T/H/P",   {  5,   3,   1},  1200,                                0},
  {"BME690 gas",     { 20,  10,   5},  11000,                               0},
  {"CM1106 CO2",     { 30,  30,  30},  4000,                                0},
  {"SEN0466 CO",     { 30,  10,   5},  500,                                 0},
  {"BMV080 PM",      { 30,  10,   5},  8000,                                0},
  {"Calypso viento", {  5,   5,   5},  CALYPSO_BOOT_MS + CALYPSO_WINDOW_MS, 0},
  {"ENVIO",          { 30,  15,   3},  500,                                 60000},
};

// Duracion medida de cada lectura en el banco (C3, 2026-10-06), solo como
// referencia: BME690 0,04 s | CM1106 3,5 s | BMV080 2,1 s | Calypso 1,5 s |
// SEN0466 0,02 s. Entre cada uno, 1,5 s de reposo. Un barrido completo: 19,3 s.

uint32_t periodoMs(uint8_t t) { return (uint32_t)TRABAJOS[t].periodoMin[estado] * 60000UL; }
uint32_t desfaseMs(uint8_t t) { return (estado == NORMAL) ? TRABAJOS[t].desfaseMs : 0; }

// El siguiente multiplo de `periodo` (mas el desfase) ESTRICTAMENTE despues de
// `desde`. Es lo que alinea las medidas a minutos redondos: con 5 min caen en
// :00, :05, :10... y con la hora real, los tres nodos miden en el mismo instante.
uint64_t siguienteMultiplo(uint64_t desde, uint32_t periodo, uint32_t desfase) {
  uint64_t base = (desde >= desfase) ? desde - desfase : 0;
  return (base / periodo + 1) * periodo + desfase;
}

// Rehace el calendario entero. Al arrancar, al cambiar de estado de alarma (cambian
// los periodos) y al recibir la primera hora de red (cambia la base de tiempo).
void replanificar(const char *motivo) {
  const uint64_t ahora = ahoraMs();
  for (uint8_t t = 0; t < NUM_TRABAJOS; t++) {
    proximoVencimientoMs[t] = siguienteMultiplo(ahora, periodoMs(t), desfaseMs(t));
  }
  Serial.printf("[calendario] replanificado (%s), estado %u\n", motivo, estado);
}

// Que trabajos le tocan a ESTE despertar. Uno vence si ya esta dentro de su
// margen de preparacion: si al BMV080 le faltan 3 s y necesita 8, es ahora.
bool tocaAhora[NUM_TRABAJOS];

void evaluarCalendario() {
  const uint64_t ahora = ahoraMs();
  uint64_t primero = UINT64_MAX;
  for (uint8_t t = 0; t < NUM_TRABAJOS; t++) {
    tocaAhora[t] = (ahora + TRABAJOS[t].preparacionMs >= proximoVencimientoMs[t]);
    if (tocaAhora[t] && proximoVencimientoMs[t] < primero) primero = proximoVencimientoMs[t];
  }
  // Agrupar los que vencen en el MISMO instante aunque su preparacion sea distinta.
  // Sin esto, el T/H/P y el viento (los dos cada 5 min) irian en dos despertares
  // seguidos, porque el viento pide 21,5 s de antelacion y el T/H/P 1,2.
  if (primero != UINT64_MAX) {
    for (uint8_t t = 0; t < NUM_TRABAJOS; t++) {
      if (proximoVencimientoMs[t] <= primero + 2000) tocaAhora[t] = true;
    }
  }
}

// Un trabajo hecho pasa a su siguiente multiplo, contado DESDE SU VENCIMIENTO y no
// desde "ahora". Como se ejecuta antes de su minuto (por la preparacion), contar
// desde "ahora" devolveria el mismo minuto y el trabajo se repetiria en el acto.
void marcarHecho(uint8_t t) {
  const uint64_t ahora = ahoraMs();
  uint64_t base = (proximoVencimientoMs[t] > ahora) ? proximoVencimientoMs[t] : ahora;
  proximoVencimientoMs[t] = siguienteMultiplo(base, periodoMs(t), desfaseMs(t));
}

// Cuantos segundos dormir: hasta el proximo despertar, que es el vencimiento mas
// cercano menos su preparacion.
uint32_t segundosHastaDespertar() {
  const uint64_t ahora = ahoraMs();
  uint64_t despertar = UINT64_MAX;
  for (uint8_t t = 0; t < NUM_TRABAJOS; t++) {
    uint64_t prep = TRABAJOS[t].preparacionMs;
    uint64_t d = (proximoVencimientoMs[t] > prep) ? proximoVencimientoMs[t] - prep : 0;
    if (d < despertar) despertar = d;
  }
  double realMs = (despertar > ahora) ? (double)(despertar - ahora) : 1000.0;

  // De tiempo REAL a tiempo del NODO: para dormir X ms reales hay que pedirle al
  // oscilador X / deriva de los suyos. Esta division es la que convierte la deriva
  // aprendida en puntualidad; sin ella se mediria y no serviria para nada.
  double nodoMs = realMs / (double)deriva;
  if (nodoMs < 1000.0) nodoMs = 1000.0;
  if (nodoMs > 3600000.0) nodoMs = 3600000.0;
  return (uint32_t)((nodoMs + 500.0) / 1000.0);
}

// ===========================================================================
// 4. ALIMENTACION: los raíles
// ===========================================================================
//
// Los sensores no estan conectados directamente a la bateria: cuelgan de dos
// transistores que el ESP32 abre y cierra. Mientras duerme, estan APAGADOS de
// verdad. OJO: polaridad OPUESTA. 3V3 (GPIO10) se enciende con LOW; 5V (GPIO11),
// con HIGH.

bool railesEncendidos = false;
uint32_t railesEncendidosMs = 0;  // cuando se encendieron (el Calypso cuenta desde aqui)
bool bmeListo = false, coListo = false, pmListo = false, co2Listo = false, vientoListo = false;

void railesSensores(bool encender) {
  pinMode(PIN_PCB_EN_A, OUTPUT);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  digitalWrite(PIN_PCB_EN_A, encender ? PIN_PCB_EN_A_ACTIVE : !PIN_PCB_EN_A_ACTIVE);
  digitalWrite(PIN_PCB_EN_B, encender ? PIN_PCB_EN_B_ACTIVE : !PIN_PCB_EN_B_ACTIVE);
  railesEncendidos = encender;
  if (encender) {
    railesEncendidosMs = millis();
    delay(500);  // dar tiempo a que la tension se estabilice
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  } else {
    // Sin corriente, cada sensor pierde su configuracion: habra que arrancarlo otra vez.
    bmeListo = coListo = pmListo = co2Listo = vientoListo = false;
  }
}

void railRadio(bool encender) {
  pinMode(PIN_LORA_EN, OUTPUT);
  digitalWrite(PIN_LORA_EN, encender ? PIN_LORA_EN_ACTIVE : !PIN_LORA_EN_ACTIVE);
  if (encender) delay(PIN_LORA_EN_SETTLE_MS);
}

// ===========================================================================
// 5. SENSORES: un trabajo cada vez
// ===========================================================================
//
// NUNCA dos sensores a la vez: juntos, la alimentacion cae y la placa se reinicia
// (31 reinicios en paralelo frente a 2 en secuencia). Cada sensor se arranca solo
// cuando le toca por primera vez desde que tiene corriente.

Valores fila;  // lo medido en ESTE despertar

void limpiar(Valores &v) {
  v.temp = v.hum = v.pres = v.gas = SIN_DATO;
  v.pm1 = v.pm25 = v.pm10 = SIN_DATO;
  v.co2 = v.co = v.coTemp = SIN_DATO;
  v.windAngle = v.windSpeed = SIN_DATO;
  v.windValid = false;
}

void leerTrabajo(uint8_t t) {
  switch (t) {
    case T_BME_THP: {  // rapida: sin calefactor, unos milisegundos
      if (!bmeListo) bmeListo = bme690.begin();
      if (!bmeListo) break;
      Reading r = bme690.readFast();
      if (r.ok()) {
        fila.temp = ultimo.temp = r.values[0];
        fila.hum  = ultimo.hum  = r.values[1];
        fila.pres = ultimo.pres = r.values[2];
      }
      break;
    }
    case T_BME_VOC: {  // completa: calefactor a 320 C para medir el gas
      if (!bmeListo) bmeListo = bme690.begin();
      if (!bmeListo) break;
      Reading r = bme690.read();
      if (r.ok()) {
        fila.temp = ultimo.temp = r.values[0];
        fila.hum  = ultimo.hum  = r.values[1];
        fila.pres = ultimo.pres = r.values[2];
        fila.gas  = ultimo.gas  = r.values[3];
      }
      break;
    }
    case T_CM1106: {  // solo mide al encenderse: cada valor nuevo exige un ciclo de alimentacion
      if (!co2Listo) { cm1106.begin(); co2Listo = true; }
      cm1106.powerCycle();
      Reading r = cm1106.read();
      if (r.ok()) fila.co2 = ultimo.co2 = r.values[0];
      cm1106.sleep();
      break;
    }
    case T_SEN0466: {
      if (!coListo) coListo = sen0466.begin();
      if (!coListo) break;
      Reading r = sen0466.read();
      if (r.ok()) {
        fila.co     = ultimo.co     = r.values[0];
        fila.coTemp = ultimo.coTemp = r.values[1];
      }
      break;
    }
    case T_BMV080: {  // laser de ~68 mA: encendido solo durante la medida
      if (!pmListo) {
        pinMode(PIN_BMV080_EN, OUTPUT);
        digitalWrite(PIN_BMV080_EN, PIN_BMV080_EN_ACTIVE);
        delay(BMV080_STARTUP_DELAY_MS);
        pmListo = bmv080.begin();
        if (pmListo) bmv080.stopMeasurement();
      }
      if (!pmListo || !bmv080.startMeasurement()) break;
      delay(BMV080_PREHEAT_MS);
      uint32_t t0 = millis();
      while (millis() - t0 < BMV080_WINDOW_MS) {
        Reading r = bmv080.read();
        if (r.ok()) {
          fila.pm1  = ultimo.pm1  = r.values[0];
          fila.pm25 = ultimo.pm25 = r.values[1];
          fila.pm10 = ultimo.pm10 = r.values[2];
          break;
        }
        delay(50);
      }
      bmv080.stopMeasurement();  // SIEMPRE, se haya medido o no
      break;
    }
    case T_CALYPSO: {  // ultrasonico: ~20 s de arranque desde que recibe corriente
      if (!vientoListo) { calypso.begin(); vientoListo = true; }
      uint32_t encendido = millis() - railesEncendidosMs;
      if (encendido < CALYPSO_BOOT_MS) delay(CALYPSO_BOOT_MS - encendido);
      calypso.flushInput();
      uint32_t t0 = millis();
      while (millis() - t0 < CALYPSO_WINDOW_MS) {
        Reading r = calypso.read();
        if (r.ok()) {
          fila.windAngle = ultimo.windAngle = r.values[0];
          fila.windSpeed = ultimo.windSpeed = r.values[1];
          fila.windValid = ultimo.windValid = r.values[2] > 0.5f;
          break;
        }
        delay(10);
      }
      break;
    }
  }
}

// Ejecuta, en fila, los trabajos de sensor que tocan. Devuelve true si leyo algo.
bool barrido() {
  bool alguno = false;
  for (uint8_t t = 0; t < T_ENVIO; t++) alguno |= tocaAhora[t];
  if (!alguno) return false;

  if (!railesEncendidos) railesSensores(true);
  limpiar(fila);

  for (uint8_t t = 0; t < T_ENVIO; t++) {
    if (!tocaAhora[t]) continue;
    uint32_t t0 = millis();
    leerTrabajo(t);
    Serial.printf("[t] %-15s %lu ms\n", TRABAJOS[t].nombre, (unsigned long)(millis() - t0));
    delay(SLOT_GAP_MS);
    marcarHecho(t);
  }

  // Guardar la fila con su momento. Si el almacen esta lleno se pierde la mas vieja.
  if (almacenN == FILAS_MAX) {
    memmove(&almacen[0], &almacen[1], sizeof(Fila) * (FILAS_MAX - 1));
    almacenN--;
  }
  almacen[almacenN].v = fila;
  almacen[almacenN].tomadaS = (uint32_t)(tiempoNodoMs() / 1000ULL);
  almacenN++;

  Serial.printf("[lectura] T=%.1f H=%.1f gas=%.0f PM2.5=%.1f CO2=%.0f CO=%.1f viento=%.1f "
                "(-999 = no tocaba)\n",
                fila.temp, fila.hum, fila.gas, fila.pm25, fila.co2, fila.co, fila.windSpeed);
  return true;
}

// ===========================================================================
// C. LA ALARMA
// ===========================================================================
//
// Cuatro umbrales por magnitud: dos para ENTRAR (preOn, almOn) y dos, algo mas
// bajos, para SALIR (preOff, almOff). Es histeresis: un valor que oscila justo en
// el umbral no hace parpadear el estado. Basta UNA magnitud para subir de nivel.

struct Regla {
  const char *nombre;
  float *valor;
  bool altoEsMalo;  // true: sube con el fuego. false: baja con el fuego (humedad)
  float preOff, preOn, almOff, almOn;
};

// Umbrales reales del proyecto (hoja de umbrales, 2026-09-28).
Regla reglas[] = {
  //  nombre       valor              alto   preOff preOn  almOff almOn
  {"temp",      &ultimo.temp,      true,   45,    50,    55,    60},
  {"rh",        &ultimo.hum,       false,  25,    20,    15,    10},
  {"pm1",       &ultimo.pm1,       true,   90,   100,   225,   250},
  {"pm25",      &ultimo.pm25,      true,   90,   100,   225,   250},
  {"pm10",      &ultimo.pm10,      true,   90,   100,   225,   250},
  {"co",        &ultimo.co,        true,    6,     8,    15,    20},
  {"windSpeed", &ultimo.windSpeed, true,    2,     5,     8,    10},
};

bool supera(const Regla &r, float v, float umbral) {
  return r.altoEsMalo ? (v >= umbral) : (v <= umbral);
}

void evaluarAlarma() {
  uint8_t nivel = NORMAL;
  for (const Regla &r : reglas) {
    float v = *r.valor;
    if (v == SIN_DATO) continue;  // un sensor sin dato no dispara nada
    uint8_t n = NORMAL;
    if (supera(r, v, r.almOn) || (estado == ALARMA && supera(r, v, r.almOff))) n = ALARMA;
    else if (supera(r, v, r.preOn) || (estado >= PREALARMA && supera(r, v, r.preOff))) n = PREALARMA;
    if (n > nivel) nivel = n;
  }

  // Confirmacion: una pre-alarma que dura 60 minutos no es ruido -> alarma.
  if (nivel == PREALARMA) {
    if (estado == NORMAL) preAlarmaDesdeMs = tiempoNodoMs();
    if (estado >= PREALARMA && tiempoNodoMs() - preAlarmaDesdeMs >= PREALARM_CONFIRM_MS) {
      nivel = ALARMA;
    }
  }

  if (nivel != estado) {
    Serial.printf("[alarma] %u -> %u\n", estado, nivel);
    estado = nivel;
    replanificar("cambio de estado");  // cambian los periodos de todos los trabajos
  }
}

// ===========================================================================
// 6. RADIO: transmitir y escuchar
// ===========================================================================

// Lo que Modulo B puede mandar en la ventana de escucha. Aqui solo la hora.
void atenderMensaje(const LoRaMessage &msg) {
  Serial.printf("[rx] %s\n", msg.payload);
  if (strncmp(msg.payload, "CFG,TIME=", 9) == 0) {
    uint32_t epocaS = strtoul(msg.payload + 9, nullptr, 10);
    sincronizarReloj(epocaS);
    char ack[32];
    int n = snprintf(ack, sizeof(ack), "ACK,TIME=%lu", (unsigned long)epocaS);
    radio.send(LORA_RX_ADDR, ack, (uint8_t)n);
  }
}

// Envia todas las filas guardadas en LOTES:
//   B,<cuantas>,<fila>;<fila>;...
// cada fila son los 13 valores + su ANTIGUEDAD en segundos:
//   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid,edad
// El nodo no necesita saber la hora para esto: Modulo B, que si la sabe, fecha
// cada fila restando la edad a la hora en que la recibe.
void transmitir() {
  railesSensores(false);  // la radio pide ~120 mA al transmitir: sin sensores encendidos
  railRadio(true);
  bool ok = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                        {7, 7, 1, 12},  // SF7, BW 125 kHz, CR 4/5, preambulo 12
                        &Serial);
  if (!ok) {
    Serial.println("[radio] FALLO al iniciar - las filas se guardan para el siguiente envio");
    railRadio(false);
    return;
  }
  radio.setTxPower(LORA_TX_POWER_DBM);

  // Un paquete LoRa admite 240 bytes: si no caben todas las filas, se reparten.
  const uint32_t ahoraS = (uint32_t)(tiempoNodoMs() / 1000ULL);
  uint8_t enviadas = 0;
  while (enviadas < almacenN) {
    char paquete[241];
    int len = snprintf(paquete, sizeof(paquete), "B,%u,", (unsigned)(almacenN - enviadas));
    uint8_t primera = enviadas;
    while (enviadas < almacenN) {
      const Fila &f = almacen[enviadas];
      char una[120];
      int n = snprintf(una, sizeof(una),
                       "%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%.0f,%.1f,%d,%lu;",
                       f.v.temp, f.v.hum, f.v.pres, f.v.gas, f.v.pm1, f.v.pm25, f.v.pm10,
                       f.v.co2, f.v.co, f.v.coTemp, f.v.windAngle, f.v.windSpeed,
                       (int)f.v.windValid, (unsigned long)(ahoraS - f.tomadaS));
      if (len + n >= (int)sizeof(paquete)) break;  // este paquete esta lleno
      memcpy(paquete + len, una, n + 1);
      len += n;
      enviadas++;
    }
    if (enviadas == primera) break;  // no deberia pasar: una fila siempre cabe
    bool enviado = radio.send(LORA_RX_ADDR, paquete, (uint8_t)len);
    Serial.printf("[tx] %s (%s)\n", paquete, enviado ? "enviado" : "FALLO");
  }
  almacenN = 0;

  // Ventana de escucha: el UNICO momento en que el nodo oye a Modulo B. Va justo
  // despues de los datos, que es cuando Modulo B contesta con la hora.
  LoRaMessage msg;
  uint32_t t0 = millis();
  while (millis() - t0 < LORA_LISTEN_MS) {
    if (radio.poll(msg)) { atenderMensaje(msg); break; }
    delay(1);
  }

  // Trama de estado. Modulo B lee el campo 8 como estado de alarma de este nodo.
  char stat[64];
  int n = snprintf(stat, sizeof(stat), "STAT,0,%d,0,00000,0,-1,-1,%u",
                   (int)esp_reset_reason(), estado);
  radio.send(LORA_RX_ADDR, stat, (uint8_t)n);

  railRadio(false);
}

// ===========================================================================
// 7. SUENO PROFUNDO
// ===========================================================================

void dormir(uint32_t segundos) {
  railesSensores(false);
  railRadio(false);

  // "Enclavar" los pines: sin esto, durante el sueno quedan al aire y un
  // transistor podria volver a encender un rail y gastar bateria toda la noche.
  gpio_hold_en((gpio_num_t)PIN_PCB_EN_A);
  gpio_hold_en((gpio_num_t)PIN_PCB_EN_B);
  gpio_hold_en((gpio_num_t)PIN_LORA_EN);
  gpio_deep_sleep_hold_en();

  // El tiempo del nodo se acumula AQUI: lo que estuvo despierto + lo que va a
  // dormir. Despues de esp_deep_sleep_start() no se ejecuta nada mas.
  nodoBaseMs += (uint64_t)millis() + (uint64_t)segundos * 1000ULL;

  Serial.printf("[sueno] %lu s\n", (unsigned long)segundos);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)segundos * 1000000ULL);
  esp_deep_sleep_start();  // no vuelve: el siguiente despertar empieza en setup()
}

// ===========================================================================
// 8. PROGRAMA PRINCIPAL
// ===========================================================================

// Solo en un arranque en frio (encendido o reset): la memoria RTC trae basura.
void iniciarMemoriaRTC() {
  nodoBaseMs = 0;
  relojSincronizado = false;
  epocaMsEnSync = nodoMsEnSync = 0;
  deriva = 1.0f;
  estado = NORMAL;
  preAlarmaDesdeMs = 0;
  limpiar(ultimo);
  almacenN = 0;
  replanificar("arranque en frio");
}

// Un despertar: medir lo que toque, enviar si toca, evaluar la alarma.
void trabajar() {
  evaluarCalendario();
  if (barrido()) evaluarAlarma();
  if (tocaAhora[T_ENVIO]) {
    transmitir();
    marcarHecho(T_ENVIO);
  }
}

bool arranqueEnFrio = false;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Al despertar, soltar los pines que se enclavaron antes de dormir.
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)PIN_PCB_EN_A);
  gpio_hold_dis((gpio_num_t)PIN_PCB_EN_B);
  gpio_hold_dis((gpio_num_t)PIN_LORA_EN);

  arranqueEnFrio = (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER);
  if (arranqueEnFrio) {
    Serial.println("\n=== CHIP FOREST - Modulo C (arranque en frio) ===");
    iniciarMemoriaRTC();
  }

  trabajar();
  // El resto ocurre en loop(): dormir, o seguir despierto en alarma.
}

void loop() {
  // En alarma (o en la ventana de reflasheo tras un arranque en frio) el nodo NO
  // duerme: consulta el calendario cada 2 s y hace lo que vaya tocando.
  const bool ventanaReflasheo = arranqueEnFrio && millis() < COLD_BOOT_AWAKE_MS;
  if (estado >= PREALARMA || ventanaReflasheo) {
    delay(ALARM_LOOP_MS);
    trabajar();
    return;
  }
  dormir(segundosHastaDespertar());
}
