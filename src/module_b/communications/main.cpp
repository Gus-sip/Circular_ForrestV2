/*
 * =============================================================================
 * CHIP FOREST - MODULO B (PASARELA) - FIRMWARE DE PRODUCCION
 * =============================================================================
 *
 * Que hace esta placa:
 *   Recibe por radio LoRa las tramas de los nodos sensores (Modulo C), las
 *   interpreta, les pone hora real y las sube a ThingsBoard por la red movil NB-IoT
 *   usando MQTT. En sentido contrario, recibe ordenes de ThingsBoard (Modulo A) y se
 *   las entrega a cada nodo en su ventana de escucha, ademas de darles la hora de red.
 *   No tiene sensores propios.
 *
 *   Modulo C  --LoRa 868 MHz-->  Modulo B  --NB-IoT / MQTT-->  ThingsBoard
 *             <--CFG,...-------            <--RPC-------------
 *
 * Ademas ofrece, para el banco, una pantalla OLED con el estado y un pequeno panel
 * web por su propio punto de acceso WiFi (con descarga del historico en CSV).
 *
 * Cableado de la radio (ver Config.h):
 *   VDD del modulo -> 3V3          GND del modulo  -> GND
 *   TXD del modulo -> GPIO4 (RX)   RXD del modulo  -> GPIO5 (TX)
 *   NRST del modulo -> al aire
 *
 * Arquitectura: radio/RYLR998 (transporte LoRa), telemetry/TelemetryParser (texto
 * -> SensorSnapshot) y modem_nbiot_mqtt (subida a ThingsBoard) no saben nada unos de
 * otros ni del servidor web; este archivo es el unico sitio donde se juntan. loop()
 * va alternando el servidor web, la radio y el modem, y ninguno bloquea a los demas:
 * servir una pagina nunca hace perder un paquete, y esperar a la radio (que poll()
 * nunca hace) nunca para el servidor.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"
#include "modem_nbiot_mqtt.h"
#include "radio/RYLR998.h"
#include "telemetry/SensorSnapshot.h"
#include "telemetry/TelemetryParser.h"
#include "web/DashboardPage.h"

// -1 = sin pin de reset propio: el RESET de este modulo va unido al del propio
// ESP32, como en todas las placas SSD1306 de este proyecto.
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

RYLR998 radio(Serial1, LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD);
// UART2 - a proposito no Serial1, que ya es de la radio LoRa de arriba. Ver el
// cableado del NB-IoT en Config.h: los pines son GP44/GP43 (UART0 nativo),
// confirmados en el hardware, no los GP8/GP7 que se supusieron al principio.
ModemNBIoTMqtt modem(Serial2, NBIOT_RX_PIN, NBIOT_TX_PIN, NBIOT_EN_PIN, NBIOT_CHANNEL_PIN, NBIOT_PWRKEY_PIN);
WebServer server(80);

// La ultima lectura recibida (la que muestra el panel web y la OLED).
SensorSnapshot g_latest;

// Anillo con cada lectura LoRa interpretada con exito, solo en RAM (se pierde al
// reiniciar o al quedarse sin corriente) - ver LORA_HISTORY_CAPACITY en Config.h.
// Independiente del anillo propio del modem: aquel existe para agrupar las subidas
// por NB-IoT y se vacia al publicar; este es solo un registro local para /history,
// nunca se vacia y, cuando se llena, sobrescribe empezando por la mas antigua.
SensorSnapshot g_history[LORA_HISTORY_CAPACITY];
uint16_t g_historyHead = 0;
uint16_t g_historyCount = 0;

// ---- Panel web ----

// Pagina principal: el HTML de web/DashboardPage.h.
static void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

// /data: la ultima lectura y el estado del modem, en JSON, para el panel.
static void handleData() {
  uint32_t now = millis();
  bool stale = g_latest.hasData && (now - g_latest.lastHeardMs > DATA_STALE_MS);
  const char *status = !g_latest.hasData ? "no_data" : (stale ? "stale" : "ok");
  uint32_t secondsAgo = g_latest.hasData ? (now - g_latest.lastHeardMs) / 1000 : 0;

  char json[800];
  snprintf(json, sizeof(json),
           "{\"status\":\"%s\",\"secondsAgo\":%lu,\"rssi\":%d,\"snr\":%d,"
           "\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f,\"gas\":%.2f,"
           "\"pm1\":%.2f,\"pm25\":%.2f,\"pm10\":%.2f,"
           "\"co2\":%.2f,\"co\":%.2f,\"coTemp\":%.2f,"
           "\"windAngle\":%.2f,\"windSpeed\":%.2f,\"windValid\":%s,"
           "\"nbiot\":{\"state\":\"%s\",\"lastError\":\"%s\",\"rssiDbm\":%d,"
           "\"attached\":%s,\"attachedSec\":%lu,\"mqttConnected\":%s,\"mqttReconnects\":%lu,"
           "\"sent\":%lu,\"failed\":%lu,\"dropped\":%lu,"
           "\"batchReadings\":%u,\"batchSeconds\":%lu,\"ringCount\":%u}}",
           status, (unsigned long)secondsAgo, g_latest.rssi, g_latest.snr, g_latest.temp, g_latest.hum,
           g_latest.pres, g_latest.gas, g_latest.pm1, g_latest.pm25, g_latest.pm10, g_latest.co2,
           g_latest.co, g_latest.coTemp, g_latest.windAngle, g_latest.windSpeed,
           g_latest.windValid ? "true" : "false",
           modem.stateName(), modem.lastError(), modem.rssiDbm(), modem.attached() ? "true" : "false",
           (unsigned long)(modem.attachedUptimeMs() / 1000), modem.mqttConnected() ? "true" : "false",
           (unsigned long)modem.mqttReconnects(), (unsigned long)modem.packetsSent(),
           (unsigned long)modem.packetsFailed(), (unsigned long)modem.packetsDropped(),
           modem.batchReadingsTarget(), (unsigned long)modem.batchSecondsTarget(), modem.ringCount());

  server.send(200, "application/json", json);
}

// /history: todas las lecturas guardadas como CSV descargable, de la mas antigua a
// la mas reciente. millisSinceBoot permite calcular la hora real si se sabe cuando
// arranco la placa, y es monotono en todo el historico aunque esta placa no tenga
// reloj. Se construye en un buffer estatico (no en el monton de String de Arduino)
// dimensionado para LORA_HISTORY_CAPACITY filas de este ancho, con margen.
static char g_historyCsvBuf[LORA_HISTORY_CAPACITY * 110 + 256];

static void handleHistory() {
  size_t pos = 0;
  auto append = [&](const char *s) {
    size_t n = strlen(s);
    if (pos + n < sizeof(g_historyCsvBuf)) {
      memcpy(g_historyCsvBuf + pos, s, n);
      pos += n;
    }
  };

  append(
      "millisSinceBoot,senderAddr,rssi,snr,temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,"
      "windSpeed,windValid\n");

  for (uint16_t i = 0; i < g_historyCount; i++) {
    const SensorSnapshot &s = g_history[(g_historyHead + i) % LORA_HISTORY_CAPACITY];

    // UN CAMPO NO MEDIDO SALE VACIO, no como -999.
    //
    // El nodo manda -999 en los campos que esa lectura no midio - el CO2 va cada
    // 30 min y el VOC cada 20 - y volcarlo tal cual al CSV seria peor que inutil:
    // una columna con -999 entre valores reales se promedia, se grafica y
    // arruina la serie sin avisar. La celda vacia es la convencion que Excel y
    // pandas ya entienden como "aqui no hubo dato".
    char f[12][12];
    const float vals[12] = {s.temp, s.hum,  s.pres, s.gas,       s.pm1,       s.pm25,
                            s.pm10, s.co2,  s.co,   s.coTemp,    s.windAngle, s.windSpeed};
    for (int k = 0; k < 12; k++) {
      if (vals[k] < -900.0f) {
        f[k][0] = '\0';
      } else {
        snprintf(f[k], sizeof(f[k]), "%.2f", (double)vals[k]);
      }
    }

    char row[180];
    snprintf(row, sizeof(row), "%lu,%u,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d\n",
             (unsigned long)s.lastHeardMs, s.senderAddr, s.rssi, s.snr, f[0], f[1], f[2], f[3],
             f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], s.windValid ? 1 : 0);
    append(row);
  }

  server.sendHeader("Content-Disposition", "attachment; filename=\"module_b_history.csv\"");
  server.send(200, "text/csv", g_historyCsvBuf);
}

// ---- Pantalla OLED ----

// Dibuja un indicador de cobertura de 4 barras, estilo movil (crecen de izquierda a
// derecha y se rellenan hasta el nivel actual), con la esquina en (x, y), alineado
// abajo dentro de una fila de 8 px de alto. Los umbrales son la convencion habitual
// de barras de movil, sobre el rango -113..-51 dBm de parseCsq() (ver
// NbiotProtocol.cpp). rssiDbm==0 significa "aun no hay lectura de AT+CSQ" (una
// lectura real nunca es 0) y se dibuja con todas las barras vacias, en vez de
// mostrar una cobertura que no se ha medido.
static void drawSignalBars(int x, int y, int rssiDbm) {
  int level;
  if (rssiDbm == 0) {
    level = 0;
  } else if (rssiDbm >= -70) {
    level = 4;
  } else if (rssiDbm >= -85) {
    level = 3;
  } else if (rssiDbm >= -100) {
    level = 2;
  } else {
    level = 1;
  }

  const int barW = 3, gap = 1, maxH = 8;
  for (int i = 0; i < 4; i++) {
    int h = (i + 1) * 2;  // 2, 4, 6, 8 px de alto
    int bx = x + i * (barW + gap);
    int by = y + (maxH - h);
    if (i < level) {
      oled.fillRect(bx, by, barW, h, SSD1306_WHITE);
    } else {
      oled.drawRect(bx, by, barW, h, SSD1306_WHITE);
    }
  }
}

// Refresca la OLED con una vista compacta de la actividad, para saber de un vistazo
// que esta haciendo la placa sin necesitar el panel web: lo reciente de la recepcion
// LoRa y su calidad, y el estado actual de la maquina de estados NB-IoT/MQTT
// (stateName() pone literalmente "PUBLISHING" mientras se esta mandando un lote,
// "ATTACHING"/"MQTT_CONNECT" mientras conecta, "IDLE" mientras espera al siguiente
// lote - ver el enum State de modem_nbiot_mqtt.h).
static void updateOledStatus() {
  uint32_t now = millis();

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);

  oled.println("Modulo B");

  if (g_latest.hasData) {
    uint32_t ageS = (now - g_latest.lastHeardMs) / 1000;
    // La marca "RX!" dura poco (un paquete nuevo es un instante, no un flujo): lo
    // justo para que una pantalla que se refresca cada 500 ms lo llegue a mostrar.
    oled.printf("LoRa: %s hace %lus\n", ageS < 3 ? "RX!" : "inactivo", (unsigned long)ageS);
    oled.printf(" rssi %d snr %d\n", g_latest.rssi, g_latest.snr);
  } else {
    oled.println("LoRa: sin datos aun");
    oled.println("");
  }

  oled.printf("NBIoT: %s\n", modem.stateName());
  oled.printf("MQTT: %s\n", modem.mqttConnected() ? "activo" : "caido");
  oled.printf("Env %lu Err %lu Prd %lu\n", (unsigned long)modem.packetsSent(),
              (unsigned long)modem.packetsFailed(), (unsigned long)modem.packetsDropped());

  drawSignalBars(112, 0, modem.rssiDbm());  // esquina superior derecha, en la fila de "Modulo B"

  oled.display();
}

// =============================================================================
// setup(): WiFi + panel web, OLED, radio LoRa y arranque (no bloqueante) del modem
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CHIP FOREST LoRa ground-station receiver ===");

  // Punto de acceso WiFi propio, para el panel web local.
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("WiFi AP: SSID=\"%s\"  password=\"%s\"\n", WIFI_AP_SSID, WIFI_AP_PASSWORD);
  Serial.printf("Dashboard: http://%s/\n", apIP.toString().c_str());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.begin();
  Serial.printf("Web server started - reading history CSV at http://%s/history\n", apIP.toString().c_str());

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    oled.clearDisplay();
    oled.display();
    Serial.println("OLED: init OK");
  } else {
    Serial.println("OLED: init FAILED (check wiring - SDA=GPIO1, SCL=GPIO2)");
  }

  pinMode(LORA_EN_PIN, OUTPUT);
  digitalWrite(LORA_EN_PIN, LOW);  // encender el rail de LoRa (puerta Q4) - LOW = encendido, confirmado probando pines
  delay(300);

  // Los parametros del enlace TIENEN que coincidir con los de los nodos.
  Serial.println("Configuring RYLR998 (link parameters must match the sensor node):");
  bool ok = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                         {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE}, &Serial);
  Serial.println(ok ? "Radio init OK - listening for packets."
                     : "Radio init FAILED - check wiring/baud before assuming a parameter is wrong.");
  Serial.println();

  // El modem arranca aqui pero trabaja poco a poco en cada vuelta de loop() (tick()).
  Serial.println("Starting NB-IoT/MQTT uplink to ThingsBoard - non-blocking, see modem_nbiot_mqtt.h:");
  modem.begin();
  Serial.println();
}

// ---- Ultimo estado conocido de cada nodo ----

// Ultimo estado de carga de cada nodo, por direccion LoRa.
//
// Se guarda aparte del anillo de telemetria porque STAT y telemetria llegan en
// paquetes distintos en momentos distintos: un nodo transmite sus lecturas y su
// salud por separado, y cualquiera de los dos puede ser el que llegue.
struct NodeCharge {
  uint16_t addr;
  int16_t pct;
  int16_t mv;
};
static NodeCharge g_nodeCharge[8];
static uint8_t g_nodeChargeCount = 0;

static void nodeChargeRemember(uint16_t addr, int pct, int mv) {
  for (uint8_t i = 0; i < g_nodeChargeCount; i++) {
    if (g_nodeCharge[i].addr == addr) {
      g_nodeCharge[i].pct = (int16_t)pct;
      g_nodeCharge[i].mv = (int16_t)mv;
      return;
    }
  }
  if (g_nodeChargeCount < (uint8_t)(sizeof(g_nodeCharge) / sizeof(g_nodeCharge[0]))) {
    g_nodeCharge[g_nodeChargeCount].addr = addr;
    g_nodeCharge[g_nodeChargeCount].pct = (int16_t)pct;
    g_nodeCharge[g_nodeChargeCount].mv = (int16_t)mv;
    g_nodeChargeCount++;
  }
}

// Ultimo estado de incendio de cada nodo, por direccion LoRa. Misma forma que el de
// carga y por el mismo motivo: STAT y telemetria son paquetes distintos que llegan
// en momentos distintos, y cualquiera puede ser el que llegue.
struct NodeAlarm {
  uint16_t addr;
  int8_t state;
};
static NodeAlarm g_nodeAlarm[8];
static uint8_t g_nodeAlarmCount = 0;

static void nodeAlarmRemember(uint16_t addr, int state) {
  for (uint8_t i = 0; i < g_nodeAlarmCount; i++) {
    if (g_nodeAlarm[i].addr == addr) {
      g_nodeAlarm[i].state = (int8_t)state;
      return;
    }
  }
  if (g_nodeAlarmCount < (uint8_t)(sizeof(g_nodeAlarm) / sizeof(g_nodeAlarm[0]))) {
    g_nodeAlarm[g_nodeAlarmCount].addr = addr;
    g_nodeAlarm[g_nodeAlarmCount].state = (int8_t)state;
    g_nodeAlarmCount++;
  }
}

// Copia en una lectura el ultimo estado de incendio conocido de su nodo.
static void nodeAlarmAttach(SensorSnapshot &snap) {
  for (uint8_t i = 0; i < g_nodeAlarmCount; i++) {
    if (g_nodeAlarm[i].addr == snap.senderAddr) {
      snap.alarmState = g_nodeAlarm[i].state;
      return;
    }
  }
}

// Copia en una lectura el ultimo estado de carga conocido de su nodo.
static void nodeChargeAttach(SensorSnapshot &snap) {
  for (uint8_t i = 0; i < g_nodeChargeCount; i++) {
    if (g_nodeCharge[i].addr == snap.senderAddr) {
      snap.chargePct = g_nodeCharge[i].pct;
      snap.capMv = g_nodeCharge[i].mv;
      return;
    }
  }
}

// =============================================================================
// loop(): servidor web, recepcion LoRa, maquina de estados del modem y OLED
// =============================================================================
void loop() {
  server.handleClient();

  // ---- Cada trama LoRa que haya llegado ----
  LoRaMessage msg;
  while (radio.poll(msg)) {
    Serial.printf("+RCV addr=%u len=%u rssi=%d snr=%d data=\"%.*s\"\n", msg.senderAddr, msg.length,
                  msg.rssi, msg.snr, msg.length, msg.payload);

    // LO PRIMERO, antes de interpretar nada: si Modulo A tiene una orden en cola para
    // este nodo, mandarla ya. Esta subida es el unico momento en que el nodo escucha:
    // abre una ventana de ~2 s justo despues de transmitir y luego queda sordo hasta su
    // siguiente envio. Todo lo que se haga antes de este envio se come esa ventana.
    char nodeName[24];
    nbiotResolveNodeName(msg.senderAddr, nodeName, sizeof(nodeName));
    // SOLO UNA TRAMA DE TELEMETRIA ABRE VENTANA. STAT Y ACK NO.
    //
    // El orden de Modulo C es: transmitir telemetria -> mantener ~2 s la ventana de
    // escucha -> transmitir STAT. La ventana esta ENTRE esas dos tramas. Asi que
    // cuando llega aqui un STAT, el nodo ya la ha cerrado y esta transmitiendo: una
    // orden mandada al recibir el STAT se pierde seguro. Lo mismo con un ACK, que el
    // nodo manda desde dentro de la ventana y despues sale de ella.
    //
    // Disparar con cada trama recibida desperdiciaba un intento cada vez que habia una
    // orden en cola entre la telemetria de un nodo y su STAT. Visto con la peticion 0,
    // 2026-10-06:
    //
    //   +RCV addr=4 data="STAT,4922,8,0,10100,63,3120,86,0"
    //     -> downlink to NodoC-3: CFG,temp_pre_off=47,... (sent)
    //   (sin ACK; recuperado 68 s despues por el reintento)
    //
    // y con la peticion 10 el dia anterior, disparada con el propio ACK del nodo.
    //
    // Una trama de telemetria es cualquier cosa que no sea una de las respuestas o
    // estados del nodo. Se reconoce por el prefijo, que es lo que las distingue en el
    // aire: una lectura empieza por un numero, o por 'B,' si es un lote.
    const bool opensWindow =
        !(msg.length >= 5 && strncmp(msg.payload, "STAT,", 5) == 0) &&
        !(msg.length >= 4 && strncmp(msg.payload, "ACK,", 4) == 0) &&
        !(msg.length >= 5 && strncmp(msg.payload, "NACK,", 5) == 0);

    if (opensWindow) {
      // Preguntar POR DISPOSITIVO, no "hay algo pendiente": con una cola, la primera
      // orden puede ser para otro nodo que no se ha despertado, y eso no debe impedir
      // atender a este en la ventana que acaba de abrir.
      const char *cfg = modem.nodeCommandCfgFor(nodeName);
      if (cfg) {
        bool sent = radio.send(msg.senderAddr, cfg, (uint8_t)strlen(cfg));
        Serial.printf("  -> downlink to %s: %s (%s)\n", nodeName, cfg, sent ? "sent" : "SEND FAILED");
        if (sent) modem.onNodeCommandDelivered(nodeName);
      } else {
        // ---- SINCRONIZACION DE RELOJ ----
        //
        // El nodo no tiene reloj: durmiendo cuenta el tiempo con su oscilador RC,
        // que se desvia un 1-2% y acumula. Aqui si hay hora de red, asi que se la
        // bajamos. No necesita protocolo nuevo - es una clave CFG mas.
        //
        // SOLO CUANDO NO HAY COMANDO PENDIENTE. La ventana de escucha del nodo da
        // para UNA trama, asi que una orden del operador siempre tiene prioridad;
        // la hora puede esperar a la siguiente ventana sin que pase nada.
        //
        // Y no en cada trama: un nodo en alarma transmite cada 3 minutos, y
        // sincronizarlo tan seguido no mejora nada y gasta tiempo de aire, que en
        // EU868 esta limitado por ciclo de trabajo.
        static uint32_t lastTimeSyncMs[sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0])] = {0};
        const unsigned long epochS = modem.netEpochSeconds();
        if (epochS != 0) {
          int slot = -1;
          for (size_t i = 0; i < sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0]); i++) {
            if (NBIOT_NODE_NAMES[i].addr == msg.senderAddr) { slot = (int)i; break; }
          }
          const uint32_t now = millis();
          // El primer envio sale siempre: lastTimeSyncMs arranca en 0 y un nodo
          // recien encendido es justo el que mas necesita la hora.
          if (slot >= 0 && (lastTimeSyncMs[slot] == 0 ||
                            now - lastTimeSyncMs[slot] >= NODE_TIME_SYNC_MIN_GAP_MS)) {
            char timeCfg[32];
            snprintf(timeCfg, sizeof(timeCfg), "CFG,TIME=%lu", epochS);
            bool sent = radio.send(msg.senderAddr, timeCfg, (uint8_t)strlen(timeCfg));
            Serial.printf("  -> reloj a %s: %s (%s)\n", nodeName, timeCfg,
                          sent ? "enviado" : "FALLO");
            if (sent) lastTimeSyncMs[slot] = now;
          }
        }
      }
    }

    // La respuesta de un nodo a una orden, no telemetria: se pasa al modem para que
    // conteste al RPC original, y no se intenta interpretar como lectura (se tiraria
    // por "numero de campos incorrecto").
    if (msg.length >= 4 && strncmp(msg.payload, "ACK,", 4) == 0) {
      modem.onNodeCommandAck(nodeName, msg.payload);
      continue;
    }

    // El nodo entendio la orden pero no aplico nada. Se mira ANTES del interprete de
    // telemetria por el mismo motivo que el ACK: es una respuesta, no una lectura, y
    // si no se tiraria por "numero de campos incorrecto" sin dejar rastro.
    if (msg.length >= 5 && strncmp(msg.payload, "NACK,", 5) == 0) {
      modem.onNodeCommandNack(nodeName, msg.payload + 5);
      continue;
    }

    // STAT - el paquete de salud del propio nodo, no telemetria.
    //
    //   STAT,<despertar>,<motivoReinicio>,<arranques>,<bme><bmv><co2><co><viento>,<antiguedad>,<mV>,<pct>,<alarma>
    //
    // Es un paquete aparte precisamente para no romper el interprete de telemetria de
    // 13 campos, pero el estado de carga que lleva merece estar en ThingsBoard: un nodo
    // a punto de quedarse sin energia deberia decirlo donde alguien lo vea. Asi que los
    // valores se recuerdan por nodo y se adjuntan a la siguiente publicacion de
    // telemetria de ese nodo.
    if (msg.length >= 5 && strncmp(msg.payload, "STAT,", 5) == 0) {
      char buf[96];
      size_t n = msg.length < sizeof(buf) - 1 ? msg.length : sizeof(buf) - 1;
      memcpy(buf, msg.payload, n);
      buf[n] = 0;

      // Los campos 6, 7 y 8 (contando desde 0) son los mV, el porcentaje y la alarma.
      int field = 0;
      int mv = -1, pct = -1, alarm = -1;
      char *tok = strtok(buf, ",");
      while (tok) {
        if (field == 6) mv = atoi(tok);
        if (field == 7) pct = atoi(tok);
        if (field == 8) alarm = atoi(tok);  // 0 normal, 1 pre-alarma, 2 alarma
        field++;
        tok = strtok(nullptr, ",");
      }

      // ESTADO DE INCENDIO - la razon de ser de todo este camino.
      //
      // Se recuerda por nodo y se adjunta a la siguiente publicacion de telemetria de
      // ese nodo, y si esta levantado ademas se salta la espera del lote para que salga
      // YA en vez de hasta un minuto despues. Agrupar en lotes esta bien para la
      // telemetria y mal para un incendio.
      //
      // El campo 8 no existe en firmware antiguo de nodo, lo que deja alarm a -1 y se
      // informa como desconocido, no como NORMAL. Un panel que muestra NORMAL con
      // seguridad para un nodo que nunca lo dijo es peor que uno que no muestra nada.
      if (alarm >= 0) {
        nodeAlarmRemember(msg.senderAddr, alarm);
        // Las lecturas de este nodo ya estan en cola con el estado anterior - el nodo
        // manda la telemetria antes que su STAT. Hay que reetiquetarlas, o la
        // publicacion inmediata de abajo subiria justo el valor que existe para
        // corregir.
        modem.applyAlarmToQueued(msg.senderAddr, (int8_t)alarm);
        if (alarm > 0) {
          char who[32];
          nbiotResolveNodeName(msg.senderAddr, who, sizeof(who));
          Serial.printf("  *** %s EN %s - publicando YA ***\n", who,
                        alarm >= 2 ? "ALARMA" : "PRE-ALARMA");
          Serial.flush();
          modem.requestFlush();
        }
      }

      // TRES casos aqui, no dos. Un nodo que no manda los campos de carga y uno que
      // los manda como "desconocido" son problemas distintos, y juntarlos manda a
      // alguien a buscar una version de firmware que esta bien.
      //
      // pct == -1 puede ser lo NORMAL: significa que el nodo no tiene el divisor
      // calibrado o no lo lleva montado. No es firmware antiguo.
      if (field <= 7) {
        // Paquete corto de verdad: los campos de mV/porcentaje no existen.
        Serial.printf("  STAT has only %d fields - node predates the charge report\n", field);
      } else {
        // Los campos estan. Se recuerdan AUNQUE pct sea -1: los milivoltios en crudo
        // son de donde saldra la calibracion, asi que tirarlos porque el porcentaje
        // es desconocido descartaria justo el numero que permitiria calcularlo.
        nodeChargeRemember(msg.senderAddr, pct, mv);
        if (pct >= 0) {
          Serial.printf("  node health: charge %d%% (%d mV at the sense pin)\n", pct, mv);
        } else {
          Serial.printf("  node health: %d mV at the sense pin, percentage not yet "
                        "calibrated (SUPERCAP_CALIBRATED=0 on the node)\n", mv);
        }
      }
      continue;
    }

    // Un nodo manda o una lectura suelta o un LOTE de ellas. Las dos formas acaban
    // aqui; hay que mantenerlas separadas, porque meter un lote por el interprete de
    // lecturas sueltas es lo que corrompia los datos que llegaban a ThingsBoard: la
    // "B" y el numero se leian como temperatura y humedad, todos los campos se
    // desplazaban dos posiciones y la segunda lectura del lote se tiraba entera.
    SensorSnapshot batch[BATCH_RX_MAX];
    uint16_t ages[BATCH_RX_MAX] = {};
    uint8_t count = 0;
    bool isBatch = (msg.length >= 2 && msg.payload[0] == 'B' && msg.payload[1] == ',');

    if (isBatch) {
      count = TelemetryParser::parseBatch(msg.payload, msg.length, batch, BATCH_RX_MAX, ages);
      if (count == 0) {
        Serial.println("  (batch payload didn't parse - dropped)");
      } else {
        Serial.printf("  batch of %u reading(s)\n", (unsigned)count);
      }
    } else if (TelemetryParser::parse(msg.payload, msg.length, batch[0])) {
      ages[0] = 0;  // una lectura suelta es actual por definicion
      count = 1;
    }

    if (count == 0 && !isBatch) {
      Serial.println("  (payload didn't match the expected schema - dropped, see "
                      "telemetry/TelemetryParser.h)");
    }

    for (uint8_t b = 0; b < count; b++) {
      SensorSnapshot &snap = batch[b];
      snap.hasData = true;
      // Se retrasa la marca de tiempo segun la antiguedad del registro, para que una
      // lectura de un lote quede fechada cuando se TOMO, no cuando llego el paquete.
      // Las lecturas sueltas tienen antiguedad 0 y no cambian.
      // LA EDAD VIENE EN SEGUNDOS REALES, medidos por el propio nodo.
      //
      // Antes se multiplicaba por NODE_TICK_SECONDS = 10, asumiendo que el nodo
      // contaba en ticks de 10 s. El tick real eran 18,8 s - un error del 88% que
      // comprimia todo el historico casi a la mitad - y con el planificador por
      // TIEMPO los despertares pasaron a ser de 5 MINUTOS, con lo que el error se
      // fue a 30x. El rediseno empeoro el bug hasta hacerlo imposible de ignorar.
      //
      // Ahora el nodo manda cuantos SEGUNDOS hace que tomo cada lectura, contados
      // con su propio reloj y corregidos por la deriva que aprende de Modulo B.
      // Aqui no hay nada que adivinar.
      uint32_t ageMs = (uint32_t)ages[b] * 1000UL;
      snap.lastHeardMs = millis() - ageMs;
      snap.senderAddr = msg.senderAddr;
      snap.rssi = msg.rssi;
      snap.snr = msg.snr;
      nodeChargeAttach(snap);  // el ultimo STAT de este nodo, si lo hay
      nodeAlarmAttach(snap);
      g_latest = snap;
      modem.enqueue(snap);  // a la cola de subida a ThingsBoard

      // Y al historico local del panel web.
      uint16_t idx = (uint16_t)((g_historyHead + g_historyCount) % LORA_HISTORY_CAPACITY);
      g_history[idx] = snap;
      if (g_historyCount < LORA_HISTORY_CAPACITY) {
        g_historyCount++;
      } else {
        g_historyHead = (uint16_t)((g_historyHead + 1) % LORA_HISTORY_CAPACITY);
      }
    }
  }

  // ---- El modem: una pequena parte de su trabajo en cada vuelta, sin bloquear ----
  modem.tick();

  // ---- La OLED, cada 500 ms ----
  static uint32_t lastOledMs = 0;
  if (millis() - lastOledMs >= 500) {
    lastOledMs = millis();
    updateOledStatus();
  }
}
