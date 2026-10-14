/*
 * CHIP FOREST - MODULO B (pasarela)  -  VERSION SIMPLIFICADA PARA EXPLICAR
 *
 * Que hace esta placa, en una frase:
 *   escucha por LoRa las tramas de los nodos (Modulo C), les pone fecha y hora, las
 *   convierte a JSON y las sube a ThingsBoard por NB-IoT (red movil) usando MQTT.
 *   Ademas es el RELOJ de los nodos: les manda la hora de la red.
 *
 *   Modulo C  --LoRa 868 MHz-->  Modulo B  --NB-IoT / MQTT-->  ThingsBoard
 *             <--CFG,TIME------
 *
 * Flujo:
 *   setup()
 *     1. encender la radio LoRa y configurarla (mismos parametros que los nodos)
 *     2. encender el modem NB-IoT (Quectel BC660K-GL), registrarse en la red,
 *        leer la hora de red y abrir la sesion MQTT con ThingsBoard
 *   loop()
 *     3. si llega un lote de un nodo -> contestarle con la hora (si toca), fechar
 *        cada lectura y guardarla en una cola
 *     4. cada MQTT_BATCH_SECONDS (o si un nodo esta en alarma) -> publicar la cola
 *
 * Diferencias con el firmware real (src/module_b/communications/):
 *   - el real nunca se bloquea: el modem es una maquina de estados no bloqueante,
 *     para no perder tramas LoRa mientras espera al modem. Aqui las funciones del
 *     modem ESPERAN a la respuesta, que es mucho mas facil de leer y explicar.
 *   - el real ademas reintenta con espera creciente, recibe comandos desde
 *     ThingsBoard y los reenvia a los nodos, y tiene pantalla OLED y panel web.
 */

#include <Arduino.h>
#include "RYLR998.h"  // driver de la radio LoRa (el mismo que usa Modulo C)

// ---------------------------------------------------------------------------
// 1. CONFIGURACION
// ---------------------------------------------------------------------------

// Radio LoRa. Debe coincidir EXACTAMENTE con Modulo C.
#define LORA_MY_ADDR      2            // direccion de Modulo B
#define LORA_NETWORK_ID   5
#define LORA_BAND_HZ      868000000UL  // EU868
#define LORA_RX_PIN       4
#define LORA_TX_PIN       5
#define LORA_EN_PIN       13           // transistor del rail de la radio, LOW = encendido

// Modem NB-IoT (Quectel BC660K-GL), conectado al UART2.
#define NBIOT_RX_PIN      44
#define NBIOT_TX_PIN      43
#define NBIOT_EN_PIN      9            // alimentacion del modem, LOW = encendido
#define NBIOT_CHANNEL_PIN 10           // canal de alimentacion, LOW = abierto
#define NBIOT_APN         "sensor.net" // APN de la SIM

// ThingsBoard. EL TOKEN ES SECRETO: no lo subas a ningun sitio publico.
#define MQTT_HOST         "test-moduloa.home.kg"
#define MQTT_PORT         18831
#define MQTT_CLIENT_ID    "CON-1"
#define MQTT_TOKEN        "PON_AQUI_EL_TOKEN"   // token del dispositivo pasarela

// Cada cuanto se sube lo acumulado.
#define MQTT_BATCH_SECONDS 600         // 10 minutos

// Como mucho una hora por nodo cada 5 minutos: un nodo en alarma transmite cada 3
// minutos, y la banda EU868 limita el tiempo que se puede estar transmitiendo.
#define TIME_SYNC_MIN_GAP_MS 300000UL

// ---------------------------------------------------------------------------
// 2. OBJETOS Y DATOS
// ---------------------------------------------------------------------------

RYLR998 radio(Serial1, LORA_RX_PIN, LORA_TX_PIN);
HardwareSerial &modem = Serial2;

// Una lectura recibida de un nodo.
struct Lectura {
  uint16_t nodo;  // direccion LoRa del nodo que la mando
  int16_t rssi;   // intensidad de la senal recibida
  float v[12];    // temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed
  bool windValid;
  int8_t alarma;  // 0 normal, 1 pre-alarma, 2 alarma, -1 desconocido
  uint64_t tsMs;  // cuando se TOMO (ms desde 1970), o 0 si aun no hay hora de red
};

// Nombres de las 12 magnitudes, en el mismo orden que la trama de Modulo C.
// Son las claves con las que aparecen en ThingsBoard.
const char *CLAVES[12] = {"temp", "rh",  "pres", "gas",    "pm1",       "pm25",
                          "pm10", "co2", "co",   "coTemp", "windAngle", "windSpeed"};

// Cola de lecturas pendientes de subir.
#define COLA_MAX 24
Lectura cola[COLA_MAX];
uint8_t colaN = 0;

// Ultimo estado de alarma de cada nodo (llega en la trama STAT, aparte de los datos).
int8_t alarmaDeNodo[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
uint32_t ultimaHoraEnviadaMs[8] = {0};  // cuando se le mando la hora a cada nodo

bool hayQuePublicarYa = false;  // un nodo entro en alarma: no esperar al lote

// Direccion LoRa -> nombre del dispositivo en ThingsBoard.
const char *nombreNodo(uint16_t addr) {
  switch (addr) {
    case 1: return "NodoC-1";
    case 3: return "NodoC-2";
    case 4: return "NodoC-3";
    default: return "NodoDesconocido";
  }
}

// ---------------------------------------------------------------------------
// 3. LA HORA DE RED
// ---------------------------------------------------------------------------
//
// El modem sabe la hora porque se la da la red movil (AT+CCLK?). Se lee de vez en
// cuando y entre lectura y lectura se cuenta con millis(), que en esta placa es
// fiable porque nunca duerme.

bool hayHora = false;
uint64_t epocaMsEnLectura = 0;   // hora de red la ultima vez que se leyo
uint32_t millisEnLectura = 0;    // millis() en ese mismo instante

uint64_t horaRealMs() {
  if (!hayHora) return 0;
  return epocaMsEnLectura + (uint32_t)(millis() - millisEnLectura);
}

// "+CCLK: \"26/10/14,08:46:08+08\""  ->  ms desde 1970.
// OJO: este modem da la hora YA EN UTC aunque anuncie la zona (+08 = +2 h). No hay
// que restar nada: restarlo ponia todas las lecturas 2 horas atrasadas.
bool interpretarCclk(const char *linea, uint64_t &epocaMs) {
  int yy, mo, dd, hh, mi, ss;
  const char *p = strchr(linea, '"');
  if (!p || sscanf(p + 1, "%d/%d/%d,%d:%d:%d", &yy, &mo, &dd, &hh, &mi, &ss) != 6) return false;
  if (yy < 23) return false;  // antes de 2023: el modem aun no ha recibido la hora

  // Dias desde 1970-01-01 (algoritmo days_from_civil, sin librerias de fechas).
  int y = 2000 + yy - (mo <= 2);
  int era = y / 400;
  int yoe = y - era * 400;
  int doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + dd - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long dias = (long)era * 146097 + doe - 719468;

  epocaMs = ((uint64_t)dias * 86400ULL + hh * 3600ULL + mi * 60ULL + ss) * 1000ULL;
  return true;
}

// Escribe un uint64 en decimal. printf("%llu") no es fiable con la libreria C de
// este compilador, y ya dio un fallo en el firmware real.
void u64aTexto(uint64_t v, char *out) {
  char tmp[21];
  int n = 0;
  do { tmp[n++] = '0' + (v % 10); v /= 10; } while (v);
  for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  out[n] = '\0';
}

// ---------------------------------------------------------------------------
// 4. INTERPRETAR LAS TRAMAS DE LOS NODOS
// ---------------------------------------------------------------------------

// Un lote: "B,<cuantas>,<fila>;<fila>;..."  Cada fila son 14 campos:
//   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid,edad
// La EDAD son los segundos que hace que el nodo tomo esa lectura. Con ella se fecha:
//   hora en que se tomo = hora de ahora - edad
// Una fila que no tenga EXACTAMENTE 14 campos se descarta entera: es mejor perder
// una lectura que guardarla con los valores desplazados de columna.
void guardarEnCola(const Lectura &l) {
  if (colaN == COLA_MAX) {  // cola llena: se pierde la mas antigua
    memmove(&cola[0], &cola[1], sizeof(Lectura) * (COLA_MAX - 1));
    colaN--;
  }
  cola[colaN++] = l;
}

void interpretarLote(const LoRaMessage &msg) {
  char buf[241];
  strncpy(buf, msg.payload, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  // Saltar la cabecera "B,<cuantas>,". El numero no se cree: se cuentan las filas
  // que de verdad llegan, por si el paquete viene cortado.
  char *p = strchr(buf, ',');
  if (p) p = strchr(p + 1, ',');
  if (!p) return;
  p++;

  const uint64_t ahora = horaRealMs();
  uint8_t filas = 0;
  for (char *fila = strtok(p, ";"); fila; fila = strtok(nullptr, ";")) {
    Lectura l = {};
    char *campos[14];
    int n = 0;
    for (char *c = fila; c && n < 15;) {
      if (n < 14) campos[n] = c;
      n++;
      c = strchr(c, ',');
      if (c) *c++ = '\0';
    }
    if (n != 14) continue;

    for (int k = 0; k < 12; k++) l.v[k] = atof(campos[k]);
    l.windValid = atoi(campos[12]) != 0;
    uint32_t edadS = strtoul(campos[13], nullptr, 10);
    l.tsMs = ahora ? ahora - (uint64_t)edadS * 1000ULL : 0;
    l.nodo = msg.senderAddr;
    l.rssi = msg.rssi;
    l.alarma = (msg.senderAddr < 8) ? alarmaDeNodo[msg.senderAddr] : -1;
    guardarEnCola(l);
    filas++;
  }
  Serial.printf("  lote de %u lectura(s)\n", filas);
}

// Trama de estado: "STAT,...,<alarma>". El campo 8 es el estado de alarma del nodo.
void interpretarStat(uint16_t addr, const char *texto) {
  char buf[96];
  strncpy(buf, texto, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  int campo = 0;
  for (char *tok = strtok(buf, ","); tok; tok = strtok(nullptr, ",")) {
    if (campo == 8 && addr < 8) {
      int alarma = atoi(tok);
      alarmaDeNodo[addr] = (int8_t)alarma;
      if (alarma > 0) {
        Serial.printf("  *** %s en %s - se publica YA ***\n", nombreNodo(addr),
                      alarma >= 2 ? "ALARMA" : "PRE-ALARMA");
        hayQuePublicarYa = true;
      }
    }
    campo++;
  }
}

// Sincronizacion de reloj: el nodo solo escucha unos 2 s justo despues de mandar
// sus datos, asi que la hora se le manda NADA MAS recibirlos, antes de hacer nada.
void mandarHoraSiToca(uint16_t addr) {
  if (!hayHora || addr >= 8) return;
  const uint32_t ahora = millis();
  if (ultimaHoraEnviadaMs[addr] != 0 && ahora - ultimaHoraEnviadaMs[addr] < TIME_SYNC_MIN_GAP_MS) {
    return;
  }
  char cfg[32];
  int n = snprintf(cfg, sizeof(cfg), "CFG,TIME=%lu", (unsigned long)(horaRealMs() / 1000ULL));
  bool ok = radio.send(addr, cfg, (uint8_t)n);
  Serial.printf("  -> hora a %s: %s (%s)\n", nombreNodo(addr), cfg, ok ? "enviada" : "FALLO");
  if (ok) ultimaHoraEnviadaMs[addr] = ahora;
}

void procesarTramaLoRa(const LoRaMessage &msg) {
  Serial.printf("+RCV nodo=%u rssi=%d \"%s\"\n", msg.senderAddr, msg.rssi, msg.payload);

  if (strncmp(msg.payload, "STAT,", 5) == 0) {  // estado del nodo
    interpretarStat(msg.senderAddr, msg.payload);
  } else if (strncmp(msg.payload, "ACK,", 4) == 0) {  // el nodo confirma la hora
    Serial.println("  (confirmacion del nodo)");
  } else if (strncmp(msg.payload, "B,", 2) == 0) {  // lote de lecturas
    mandarHoraSiToca(msg.senderAddr);  // PRIMERO: la ventana de escucha se cierra en 2 s
    interpretarLote(msg);
  } else {
    Serial.println("  (formato desconocido - descartada)");
  }
}

// ---------------------------------------------------------------------------
// 5. MODEM NB-IoT: comandos AT
// ---------------------------------------------------------------------------
//
// El modem se maneja con comandos de texto "AT..." por el puerto serie. Cada
// comando contesta "OK" o "ERROR", y algunos mandan despues una respuesta propia
// (por ejemplo "+QMTCONN: 0,0,0" cuando la conexion MQTT termina).

// Espera hasta ver una linea que empiece por `esperado` (o por `alternativa`, si
// se da). Si `copia` no es nulo, guarda ahi la linea encontrada.
bool esperarRespuesta(const char *esperado, uint32_t timeoutMs,
                      const char *alternativa = nullptr, String *copia = nullptr) {
  String linea;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (modem.available()) {
      char c = modem.read();
      if (c == '\n') {
        linea.trim();
        if (linea.length()) Serial.printf("  <- %s\n", linea.c_str());
        if (linea.startsWith(esperado) || (alternativa && linea.startsWith(alternativa))) {
          if (copia) *copia = linea;
          return true;
        }
        if (linea.indexOf("ERROR") >= 0) return false;
        linea = "";
      } else {
        linea += c;
      }
    }
    delay(1);
  }
  Serial.println("  (sin respuesta a tiempo)");
  return false;
}

// Manda un comando y espera su respuesta.
bool comandoAT(const char *cmd, const char *esperado = "OK", uint32_t timeoutMs = 5000,
               const char *alternativa = nullptr, String *copia = nullptr) {
  while (modem.available()) modem.read();  // vaciar lo que hubiera
  modem.print(cmd);
  modem.print("\r\n");
  Serial.printf("  -> %s\n", cmd);
  return esperarRespuesta(esperado, timeoutMs, alternativa, copia);
}

// Lee la hora de la red y la deja apuntada junto con el millis() del momento.
void leerHoraRed() {
  String linea;
  uint64_t epocaMs;
  if (comandoAT("AT+CCLK?", "+CCLK:", 5000, nullptr, &linea) &&
      interpretarCclk(linea.c_str(), epocaMs)) {
    epocaMsEnLectura = epocaMs;
    millisEnLectura = millis();
    hayHora = true;
    Serial.printf("[hora] hora de red: %lu s desde 1970\n", (unsigned long)(epocaMs / 1000ULL));
  }
}

bool mqttConectado = false;

// Enciende el modem, se registra en la red y abre la sesion MQTT.
bool conectarModem() {
  mqttConectado = false;

  // Encender: abrir el canal y dar alimentacion.
  pinMode(NBIOT_EN_PIN, OUTPUT);
  pinMode(NBIOT_CHANNEL_PIN, OUTPUT);
  digitalWrite(NBIOT_CHANNEL_PIN, LOW);
  digitalWrite(NBIOT_EN_PIN, LOW);
  delay(3000);

  // El UART se abre DESPUES de dar corriente: si se abre antes, la linea TX
  // alimenta al modem por la puerta de atras y no arranca bien.
  modem.begin(115200, SERIAL_8N1, NBIOT_RX_PIN, NBIOT_TX_PIN);

  // Esperar a que conteste (puede tardar bastantes segundos tras encender).
  bool vivo = false;
  for (int i = 0; i < 30 && !vivo; i++) vivo = comandoAT("AT", "OK", 1000);
  if (!vivo) return false;

  comandoAT("ATE0");       // sin eco de los comandos
  comandoAT("AT+CMEE=2");  // errores con texto, no solo "ERROR"
  comandoAT("AT+CTZU=1");  // que el modem actualice su hora con la de la red

  // Configurar la SIM y el APN. El APN solo se acepta con la radio apagada.
  comandoAT("AT+CFUN=0");
  char cmd[160];
  snprintf(cmd, sizeof(cmd), "AT+QCGDEFCONT=\"IP\",\"%s\"", NBIOT_APN);
  comandoAT(cmd);
  comandoAT("AT+CFUN=1");
  if (!comandoAT("AT+CPIN?", "+CPIN: READY")) return false;  // ¿SIM lista?

  // Esperar al registro en la red. +CEREG: 0,1 = red propia, 0,5 = itinerancia.
  bool registrado = false;
  for (int i = 0; i < 30 && !registrado; i++) {
    registrado = comandoAT("AT+CEREG?", "+CEREG: 0,1", 2000, "+CEREG: 0,5");
    if (!registrado) delay(2000);
  }
  if (!registrado) return false;

  leerHoraRed();  // con la red ya registrada, el modem tiene la hora

  // MQTT. Abrir la conexion con el servidor y autenticarse con el token.
  snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_HOST, MQTT_PORT);
  if (!comandoAT(cmd, "+QMTOPEN: 0,0", 15000)) return false;

  snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\"", MQTT_CLIENT_ID, MQTT_TOKEN);
  if (!comandoAT(cmd, "+QMTCONN: 0,0,0", 15000)) return false;

  mqttConectado = true;
  Serial.println("[mqtt] conectado a ThingsBoard");
  return true;
}

// Publica un mensaje JSON en un topic. El modem contesta '>' pidiendo el texto,
// que se termina con Ctrl+Z (0x1A).
bool publicar(const char *topic, const char *json) {
  char cmd[96];
  snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,0,0,0,\"%s\"", topic);
  while (modem.available()) modem.read();
  modem.print(cmd);
  modem.print("\r\n");

  uint32_t t0 = millis();  // esperar el '>'
  while (millis() - t0 < 5000) {
    if (modem.available() && modem.read() == '>') break;
    delay(1);
  }
  modem.print(json);
  modem.write((uint8_t)0x1A);

  // Confirmacion de envio: "+QMTPUB: 0,0,0".
  return esperarRespuesta("+QMTPUB: 0,0,0", 10000);
}

// ---------------------------------------------------------------------------
// 6. SUBIR LA COLA A THINGSBOARD
// ---------------------------------------------------------------------------
//
// Modulo B es una PASARELA en ThingsBoard: un solo dispositivo que publica en
// nombre de varios. Por eso usa el topic v1/gateway/telemetry y el JSON va
// agrupado por nombre de nodo. Cada lectura lleva su hora ("ts", en ms) para que
// ThingsBoard la ponga en su momento real y no en el de la subida:
//
//   {"NodoC-3":[{"ts":1760430368000,"values":{"temp":21.5,"rh":40.2,...}}, ...]}
//
// Un valor -999 significa "no se midio" y NO se publica: si se publicara,
// ThingsBoard lo dibujaria como una medida real de -999.

void publicarCola() {
  if (colaN == 0) return;

  static char json[3072];
  size_t n = 0;
  n += snprintf(json + n, sizeof(json) - n, "{");

  bool primerNodo = true;
  for (uint16_t addr = 0; addr < 8; addr++) {
    bool primeraLectura = true;
    for (uint8_t i = 0; i < colaN; i++) {
      const Lectura &l = cola[i];
      if (l.nodo != addr) continue;

      if (primeraLectura) {
        n += snprintf(json + n, sizeof(json) - n, "%s\"%s\":[",
                      primerNodo ? "" : ",", nombreNodo(addr));
        primerNodo = false;
      }
      n += snprintf(json + n, sizeof(json) - n, "%s", primeraLectura ? "" : ",");
      primeraLectura = false;

      // Con hora: {"ts":...,"values":{...}}. Sin hora: {...} y ThingsBoard pone la
      // hora de llegada.
      if (l.tsMs) {
        char ts[21];
        u64aTexto(l.tsMs, ts);
        n += snprintf(json + n, sizeof(json) - n, "{\"ts\":%s,\"values\":", ts);
      }
      n += snprintf(json + n, sizeof(json) - n, "{");
      for (int k = 0; k < 12; k++) {
        if (l.v[k] < -900.0f) continue;  // no se midio
        n += snprintf(json + n, sizeof(json) - n, "\"%s\":%.2f,", CLAVES[k], l.v[k]);
      }
      n += snprintf(json + n, sizeof(json) - n,
                    "\"windValid\":%s,\"rssi\":%d,\"alarmState\":%d}",
                    l.windValid ? "true" : "false", l.rssi, l.alarma);
      if (l.tsMs) n += snprintf(json + n, sizeof(json) - n, "}");
    }
    if (!primeraLectura) n += snprintf(json + n, sizeof(json) - n, "]");
  }
  n += snprintf(json + n, sizeof(json) - n, "}");

  Serial.printf("[mqtt] publicando %u lectura(s): %s\n", colaN, json);
  if (publicar("v1/gateway/telemetry", json)) {
    colaN = 0;  // solo se vacia la cola si el envio se confirmo
    hayQuePublicarYa = false;
  } else {
    Serial.println("[mqtt] fallo al publicar - se reconecta y se reintenta");
    mqttConectado = false;
  }
}

// ---------------------------------------------------------------------------
// 7. PROGRAMA PRINCIPAL
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CHIP FOREST - Modulo B (pasarela) ===");

  // Radio LoRa.
  pinMode(LORA_EN_PIN, OUTPUT);
  digitalWrite(LORA_EN_PIN, LOW);  // encender el rail de la radio
  delay(300);
  bool ok = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                        {7, 7, 1, 12},  // SF7, BW 125 kHz, CR 4/5, preambulo 12
                        &Serial);
  Serial.println(ok ? "[lora] escuchando" : "[lora] FALLO al iniciar");

  // Modem, hora de red y MQTT.
  if (!conectarModem()) Serial.println("[mqtt] no se pudo conectar - se reintentara");
}

void loop() {
  // 1. Recoger todas las tramas LoRa que hayan llegado. poll() no espera: si no hay
  //    nada devuelve false al momento.
  LoRaMessage msg;
  while (radio.poll(msg)) procesarTramaLoRa(msg);

  // 2. ¿Toca publicar? Por tiempo, o ya mismo si algun nodo esta en alarma.
  static uint32_t ultimaPublicacion = 0;
  bool tocaLote = millis() - ultimaPublicacion >= MQTT_BATCH_SECONDS * 1000UL;

  if (colaN > 0 && (tocaLote || hayQuePublicarYa)) {
    if (!mqttConectado) conectarModem();
    if (mqttConectado) {
      leerHoraRed();  // refrescar la hora en cada subida
      publicarCola();
    }
    ultimaPublicacion = millis();
  }

  delay(10);
}
