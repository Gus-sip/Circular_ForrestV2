#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// =============================================================================
// CONFIGURACION DE MODULO B (PASARELA LoRa -> NB-IoT -> ThingsBoard)
// =============================================================================
//
// Todo lo ajustable esta aqui, con la misma convencion que el Config.h del nodo.
//
// OJO, SEGURIDAD: este archivo contiene el token de acceso de ThingsBoard y la
// contrasena del WiFi local. No debe publicarse.

// ---------- Cableado de la radio RYLR998 ----------
// El mismo cruce que en el nodo: TXD del modulo -> RX del ESP, RXD del modulo <- TX
// del ESP. Esta placa no tiene sensores: solo el modulo LoRa.
#define LORA_RX_PIN 4  // TXD del modulo -> RX del ESP
#define LORA_TX_PIN 5  // RXD del modulo <- TX del ESP
#define LORA_BAUD 115200
#define LORA_EN_PIN 13  // puerta Q4 del rail, LOW = encendido - descubierta probando pines el 2026-08-19

// ---------- Parametros del enlace LoRa - TIENEN que coincidir exactamente con el nodo ----------
// SF/BW/CR/preambulo NO son los valores por defecto "habitualmente documentados"
// del RYLR998 (7,7,1,4) que se supusieron al principio: esa combinacion la rechazo
// con +ERR=18 el modulo del nodo. Estos valores se leyeron directamente de ese
// modulo con AT+PARAMETER? en vez de adivinarlos otra vez. Si se cambian, hay que
// cambiarlos en los dos extremos a la vez y volver a comprobarlos.
#define LORA_BAND_HZ 868000000UL  // EU868 - despliegue en Espana (confirmado, no cambiar)
#define LORA_NETWORK_ID 5
// SF7 (antes SF9) - TIENE que coincidir con LORA_PARAM_SF de chip_forest_lora_tx.cpp
// del nodo. Bajado de 9 a 7 el 2026-08-28 para que la cadencia de envio de entonces
// cumpliera el ciclo de trabajo del 1% de EU868; cuesta ~5-6 dB de enlace. Ver la
// cabecera de aquel archivo.
#define LORA_PARAM_SF 7
#define LORA_PARAM_BW 7
#define LORA_PARAM_CR 1
#define LORA_PARAM_PREAMBLE 12

// Cuantas lecturas puede llevar como mucho un paquete de lote. El nodo parte sus
// lotes para que quepan en los 240 bytes del RYLR998; esto es eso mas margen.
#define BATCH_RX_MAX 6

// NODE_TICK_SECONDS se quito el 2026-10-12. Valia 10 y servia para fechar cada
// lectura de un lote: se multiplicaba la "edad" que mandaba el nodo por esos 10 s.
//
// Era doblemente falso. El tick real del planificador viejo eran 18,8 s, no 10,
// asi que el historico salia comprimido casi a la mitad; y con el planificador por
// TIEMPO los despertares pasaron a ser de minutos, con lo que el error se fue a
// 30x. Ahora el nodo manda los SEGUNDOS reales transcurridos desde cada lectura,
// contados con su propio reloj, y aqui solo hay que multiplicar por 1000.

// Minimo entre dos sincronizaciones de reloj al MISMO nodo.
//
// La hora se baja en la ventana de escucha que el nodo abre tras cada trama de
// telemetria, pero no conviene hacerlo en todas: en alarma un nodo transmite cada
// 3 minutos y resincronizarlo tan seguido no mejora la precision - el oscilador no
// deriva nada apreciable en 3 minutos - y gasta tiempo de aire, que en EU868 esta
// limitado por ciclo de trabajo.
//
// 5 minutos deja que el estado normal (un envio cada 31 min) sincronice SIEMPRE,
// que es donde de verdad importa porque es donde el nodo duerme.
#define NODE_TIME_SYNC_MIN_GAP_MS 300000UL

#define LORA_NODE_ADDR 1  // direccion AT+ADDRESS del nodo C1
#define LORA_MY_ADDR 2    // direccion AT+ADDRESS propia de este receptor - distinta de la de los nodos

// ---------- Tabla de nombres (direccion LoRa -> nombre del dispositivo en ThingsBoard) ----------
// La usa la publicacion MQTT de pasarela para nombrar cada nodo en el contenido de
// v1/gateway/telemetry. Una entrada por nodo desplegado; anadir aqui los nodos
// nuevos y no quitar ni renumerar nunca una entrada existente (el nombre es lo que
// ThingsBoard usa para identificar el dispositivo en adelante).
struct NodeNameEntry {
  uint16_t addr;
  const char *name;
};

// La direccion 2 es este receptor (LORA_MY_ADDR) y falta a proposito: nunca emite
// telemetria. Todos los nodos salieron con la direccion 1, lo que los hacia
// indistinguibles en el aire; ahora cada uno tiene la suya, fijada en cada
// compilacion con los entornos env:module-c-node*.
static const NodeNameEntry NBIOT_NODE_NAMES[] = {
    {LORA_NODE_ADDR, "NodoC-1"},  // 1
    {3, "NodoC-2"},
    {4, "NodoC-3"},
};

// Traduce addr a su nombre configurado, o a "NodoDesconocido-<addr>" si no esta en
// la tabla de arriba. A proposito nunca se tira una lectura porque su emisor no se
// reconozca: un nodo con un nombre provisional se ve y se arregla en ThingsBoard; un
// nodo cuyos datos se tiran en silencio, no.
inline void nbiotResolveNodeName(uint16_t addr, char *outName, size_t outCap) {
  for (size_t i = 0; i < sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0]); i++) {
    if (NBIOT_NODE_NAMES[i].addr == addr) {
      strncpy(outName, NBIOT_NODE_NAMES[i].name, outCap - 1);
      outName[outCap - 1] = '\0';
      return;
    }
  }
  snprintf(outName, outCap, "NodoDesconocido-%u", addr);
}

// ---------- Pantalla OLED de estado (GME12864, compatible SSD1306, 128x64 por I2C) ----------
#define OLED_SDA_PIN 1
#define OLED_SCL_PIN 2
#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ---------- Punto de acceso WiFi ----------
// Credenciales PROVISIONALES - cambiar antes del despliegue. WPA2 exige una clave de
// 8 a 63 caracteres. (Ojo: setup() imprime la contrasena por el puerto serie.)
#define WIFI_AP_SSID "CHIP-FOREST-RX"
#define WIFI_AP_PASSWORD "chipforest1"

// ---------- Historico local ----------
// Anillo en RAM con las lecturas recibidas, descargable en CSV desde /history:
// permite registrar lo que mando Modulo C aunque Modulo A/MQTT no este accesible (o
// para una prueba rapida en el banco sin esperar a ThingsBoard). No sobrevive a un
// reinicio ni a un corte: es una comodidad de banco, no un sustituto de la subida
// real a ThingsBoard.
#define LORA_HISTORY_CAPACITY 200

// ---------- Datos viejos en el panel ----------
// Si no llega ningun paquete en este tiempo, el panel muestra "senal perdida" en vez
// de los ultimos numeros, para que un dato viejo no se confunda con uno en vivo.
// Debe quedar holgadamente por encima del periodo de envio del nodo, o el panel
// marcaria "perdido" entre dos envios normales. (OJO: se fijo cuando el nodo
// enviaba cada 5 min; con el calendario actual, en estado NORMAL el nodo envia cada
// 30 min, asi que con 12 min el panel local mostrara "stale" entre envios normales.)
#define DATA_STALE_MS 720000

// ---------- Subida NB-IoT (Quectel BC660K-GL, el enlace movil de Modulo B) ----------
// UART2 - a proposito NO Serial1 (UART1), que ya es del RYLR998 de arriba en
// GPIO4/5. El ESP32-S3 solo tiene tres controladores UART (0/1/2) y el USB-CDC va
// aparte de todos; reutilizar UART1 para el modem chocaria con la radio LoRa en
// cuanto los dos funcionaran en el mismo firmware.
#define NBIOT_BAUD 115200

// Secuencia de alimentacion y pines confirmada en el hardware el 2026-08-19 sobre la
// placa real; sustituye a todo lo que esta seccion suponia antes, que nunca se habia
// validado con los tiempos reales:
//   GP9 LOW (VIN) -> GP10 LOW (un canal con transistor que no estaba en ningun
//   esquema revisado - sin el, el UART no lleva ninguna senal) -> espera -> GP11 a
//   LOW ~1000 ms y luego HIGH (PWRKEY; al contrario de lo que se suponia, este modulo
//   NO arranca solo con VIN) -> esperar el arranque -> AT funciona en RX=GP44/TX=GP43
//   a 115200 (los pines UART0 nativos del esquema eran los buenos desde el
//   principio; el modulo simplemente nunca habia llegado a arrancar).
// (Nota posterior: segun la investigacion de octubre, PWRKEY NO esta cableado en
// esta placa; el modem arranca con VIN.)
#define NBIOT_RX_PIN 44  // RX del ESP <- TXD del modulo
#define NBIOT_TX_PIN 43  // TX del ESP -> RXD del modulo

#define NBIOT_EN_PIN 9
#define NBIOT_EN_ACTIVE LOW
#define NBIOT_DISABLE (!NBIOT_EN_ACTIVE)

#define NBIOT_CHANNEL_PIN 10  // canal con transistor, LOW = abierto - descubierto el 2026-08-19
#define NBIOT_CHANNEL_ACTIVE LOW

#define NBIOT_PWRKEY_PIN 11
#define NBIOT_PWRKEY_ACTIVE LOW
#define NBIOT_PWRKEY_PULSE_MS 1000

// RST, P1 pin 5 (el esquema llama a esta pista "RTS" por error: no es una linea de
// control de flujo del UART, es la entrada de reset del modulo). Pulso activo a
// nivel bajo. HISTORICO: no lo usa la secuencia confirmada de arriba; se dejo para
// una futura recuperacion por reset explicito que no esta hecha.
#define NBIOT_RST_PIN 6
#define NBIOT_RST_ACTIVE LOW
#define NBIOT_RST_PULSE_MS 200

// ---------- Red (SIM de 1NCE) ----------
// "iot.1nce.net" es la Plataforma 1.0 de 1NCE y en esta cuenta (Plataforma 2.0) da
// registro denegado - confirmado por el soporte de 1NCE y en la puesta en marcha del
// 2026-08-19. Usar sensor.net, PAP, usuario y contrasena vacios.
#define NBIOT_APN "sensor.net"
// El contexto PDP propio del BC660K-GL se numera desde 0, NO desde 1: la
// documentacion del BC66 (y casi todos los ejemplos AT de Quectel) usan el contexto
// 1, una trampa habitual con este modulo. IMPORTANTE: el contexto 0 es el contexto
// por defecto integrado del modulo, y el comando estandar AT+CGDCONT NO funciona con
// el (se rechaza con "+CME ERROR: operation not supported", confirmado el
// 2026-08-19): usar AT+QCGDEFCONT="IP",<apn>. Ademas AT+CFUN=1 tiene que mandarse
// expresamente antes de AT+CPIN?: este modulo no activa solo su radio al arrancar
// (AT+CPIN? falla con "+CME ERROR: ue not power on" hasta que se manda CFUN=1).
#define NBIOT_CONTEXT_ID 0

// ---------- MQTT / ThingsBoard (ModemNBIoTMqtt) ----------
// test-moduloa.home.kg es la instancia propia de ThingsBoard de Modulo A - no
// ThingsBoard Cloud ni demo.thingsboard.io, confirmado por el usuario el 2026-08-19.
//
// El puerto 1883 solo es accesible desde la red local, NO desde internet - se vio el
// 2026-08-19 comparando con la configuracion de un dispositivo ESPHome que funcionaba
// con este mismo servidor: los clientes por internet (este, por la red movil) pasan
// por un reenvio en Azure + un tunel SSH inverso en el puerto 18831. Conectarse al
// 1883 por internet no falla a nivel TCP/MQTT: devuelve un CONNACK real y bien
// formado, pero siempre con el codigo 5 ("no autorizado"), que parece exactamente un
// problema de credenciales y costo bastante depuracion antes de dar con el puerto.
// Comprobado: un cliente paho-mqtt en Python recibio CONNACK 5 en 1883 y CONNACK 0
// (correcto) en 18831 con el mismo token, sin ningun otro cambio.
#define MQTT_BROKER_HOST "test-moduloa.home.kg"
#define MQTT_BROKER_PORT 18831
// SOLO LA ETIQUETA DE LA SESION MQTT - NO EL NOMBRE DEL DISPOSITIVO, Y A PROPOSITO
// SIN CAMBIAR.
//
// El dispositivo en ThingsBoard se llama "CON-01". Este texto dice "CON-1" y es
// inofensivo: ThingsBoard identifica el dispositivo por el TOKEN DE ACCESO de abajo,
// que llega como usuario MQTT. El client id es solo el identificador de la sesion.
//
// Confirmado el 2026-10-06, despues de que la diferencia costara tiempo: el terminal
// de depuracion RPC apunta a "CON-01", no hay ningun dispositivo llamado "CON-1", y B
// llevaba todo el tiempo conectandose bien.
//
// NO se cambia a "CON-01" a proposito. La persistencia de la sesion va ligada al
// client id, asi que renombrarlo empieza una sesion nueva en el servidor y tira todo
// lo que hubiera en cola para la vieja - incluidas las ordenes publicadas mientras B
// se reinicia, que es justo la entrega de la que dependemos. La confusion sale mas
// barata documentada que los mensajes perdidos depurandolos.
#define MQTT_CLIENT_ID "CON-1"
// Autenticacion de ThingsBoard por token de acceso: el token va como usuario MQTT,
// sin contrasena. Es el token del dispositivo PASARELA - "CON-01" en ThingsBoard,
// creado el 2026-08-28 con "Is gateway" activado para sustituir al antiguo
// "CON-MODB_TEST" - y no el de ningun nodo: la pasarela publica en nombre de los
// nodos con la API de pasarela (ver modem_nbiot_mqtt.h).
// CADA PLACA MODULO B TIENE SU PROPIO DISPOSITIVO PASARELA Y SU PROPIO TOKEN, y el
// token va compilado - asi que esta linea tiene que corresponder a la placa que se
// flashea, o la telemetria acaba en silencio en el dispositivo de la otra placa sin
// ningun error en ningun extremo.
//
//   Module B #1  MAC 28:84:85:6F:8F:DC   QRPJgyk5COJPCavycmpp   <-- set below
//   Module B #2  MAC 28:84:85:6f:88:94   lIdRdiuBluhDbApBaPmJ
//
// Comprobar la MAC antes de flashear (esptool la imprime, y pyserial la da como
// serial_number). Las MAC de las dos placas solo se diferencian en los tres ultimos
// bytes.
#define MQTT_ACCESS_TOKEN "QRPJgyk5COJPCavycmpp"
#define MQTT_CLIENT_IDX 0  // indice de cliente de AT+QMTOPEN/QMTCONN/QMTPUB - solo hay un cliente MQTT abierto, asi que 0 fijo

// ---------- Lotes / cadencia de publicacion MQTT (ModemNBIoTMqtt) ----------
// Independiente de las constantes NBIOT_BATCH_* de la trama binaria de mas abajo
// (esas son de la via UDP retirada y de su prueba unitaria). La via MQTT acumula las
// lecturas LoRa y las sube en lotes por tiempo:
//
//   MQTT_BATCH_SECONDS       - publicar cada este tiempo.
//   MQTT_BATCH_MAX_READINGS  - disparo por cantidad: publicar antes si el anillo
//                              llega a estas lecturas antes del temporizador (evita
//                              que una subida lenta haga desbordar el anillo).
//   MQTT_PUB_CHUNK_READINGS  - un solo AT+QMTPUB tiene que quedar por debajo del
//                              limite de ~1400 bytes por mensaje del BC660K-GL; un
//                              lote se manda como ceil(N/trozo) QMTPUB seguidos por la
//                              sesion ya abierta. Una lectura con marca de tiempo
//                              son ~230 bytes normalmente y ~300 en el peor caso, asi
//                              que 4 (~1200 B en el peor caso) deja margen bajo 1400.
// Bajado de 600 a 60 (2026-09-14). Diez minutos es mucho tiempo para quedarse una
// lectura de un sensor de INCENDIOS, y ademas hacia imposible comprobar la subida:
// con menos de MQTT_BATCH_MAX_READINGS en cola no se publicaba nada en diez minutos y
// la cadena parecia rota cuando solo estaba esperando. 60UL es MQTT_BATCH_SECONDS_MIN.
// Subirlo otra vez si el volumen de datos NB-IoT pasa a ser el limite: se cambia
// coste de subida por rapidez de deteccion.
#define MQTT_BATCH_SECONDS 60UL
#define MQTT_BATCH_MAX_READINGS 6
#define MQTT_PUB_CHUNK_READINGS 4

// Los dos objetivos de lote se pueden cambiar en marcha desde Modulo A - ver la
// seccion de ordenes de abajo. Se recortan para que una orden mal tecleada no pueda
// parar la telemetria ni agotar los datos de la SIM:
//   suelo de SEGUNDOS 60  - por debajo, la subida cuesta mas en sobrecarga movil de lo
//                           que valen las lecturas.
//   techo de LECTURAS     - nunca por encima de NBIOT_RING_CAPACITY, o el disparo por
//                           cantidad no llegaria nunca antes de que el anillo diera la
//                           vuelta y tirara lecturas.
#define MQTT_BATCH_SECONDS_MIN 60UL
#define MQTT_BATCH_SECONDS_MAX 86400UL
#define MQTT_BATCH_READINGS_MIN 1
#define MQTT_BATCH_READINGS_MAX NBIOT_RING_CAPACITY

// ---------- Ordenes: Modulo A -> Modulo B (-> Modulo C) ----------
// Dos suscripciones, porque ThingsBoard se dirige a la propia pasarela y a sus
// dispositivos hijos por topics distintos:
//   MQTT_TOPIC_DEVICE_RPC_SUB  - RPC a ESTE dispositivo (la pasarela CON-01): ajustes
//                                de Modulo B y la orden configurarModuloC. La
//                                respuesta va a MQTT_TOPIC_DEVICE_RPC_RESP + el id.
//   MQTT_TOPIC_GATEWAY_RPC     - RPC a un dispositivo hijo (NodoC-1, etc.). Modulo B
//                                lo traduce a la gramatica LoRa "CFG,<clave>=<valor>"
//                                que ya entiende Modulo C, lo pone en cola y lo manda
//                                en la siguiente ventana de escucha de ese nodo. La
//                                respuesta se publica en este mismo topic.
#define MQTT_TOPIC_DEVICE_RPC_SUB "v1/devices/me/rpc/request/+"
#define MQTT_TOPIC_DEVICE_RPC_RESP "v1/devices/me/rpc/response/"
#define MQTT_TOPIC_GATEWAY_RPC "v1/gateway/rpc"

// Donde va el commandLog. El protocolo acordado (Protocolo_comandos_ModA_hacia_
// ModC_por_B_v1) informa del RESULTADO de una orden como telemetria y no como
// respuesta RPC, porque el RPC se contesta en el acto con "enviado" mucho antes de
// llegar al nodo. Dos mensajes distintos, dos topics distintos.
#define MQTT_TOPIC_DEVICE_TELEMETRY "v1/devices/me/telemetry"
#define MQTT_DOWNLINK_QOS 1

// Una orden para un nodo solo se puede entregar en la ventana de escucha de ~2 s
// tras cada envio del nodo. Pasado este tiempo se deja de intentar y se informa de
// un timeout, en vez de mantener la orden en cola para siempre contra un nodo muerto.
// (OJO: "3 ventanas perdidas a 5 min" era con el envio cada 5 min; con el calendario
// actual, en estado NORMAL el nodo envia cada 30 min, asi que una orden puede expirar
// antes de que el nodo abra su siguiente ventana.)
#define DOWNLINK_QUEUE_TIMEOUT_MS 900000UL  // 15 min = 3 missed windows at 5 min

// Separacion minima entre dos intentos de entrega de LA MISMA orden.
//
// Un nodo transmite mas de una vez por ventana - telemetria, y el STAT unos 2 s
// despues - y main.cpp ejecuta su gancho de ordenes con cada trama que recibe. Sin un
// minimo aqui el reintento saltaria otra vez con la trama STAT, gastando todos los
// intentos dentro de una misma ventana cuando el nodo ya estaba sordo.
//
// 10 s queda por encima de esa separacion dentro de la ventana y por debajo de
// LORA_TX_PERIOD_MIN_MS (15 s), el periodo de envio mas corto que se le puede poner a
// un nodo, asi que una ventana nunca produce dos intentos y nunca se salta una.
#define DOWNLINK_RETRY_MIN_GAP_MS 10000UL

// Marcas de tiempo por lectura: sin "ts", ThingsBoard pone a todas las lecturas de un
// lote la hora de llegada, aplastando un lote de minutos en un instante. Modulo B no
// tiene reloj, asi que lee la hora de red una vez por registro con AT+CCLK? (necesita
// AT+CTZU=1, puesto en CONFIG) y fecha cada lectura. Si la red no da nunca la hora
// (ano de CCLK < 2023), la subida pasa sola a la forma plana sin "ts".
#define NBIOT_MIN_VALID_EPOCH_MS 1672531200000LL  // 2023-01-01T00:00:00Z - anterior = "sin hora de red"

// KEEPALIVE: corto, porque Modulo B tiene que estar ESCUCHANDO, no solo accesible.
//
// Valia 3600 s - el maximo - con el razonamiento de que hacer ping entre envios
// reales era un desperdicio: "un ritmo de PINGREQ lo bastante corto para importar
// (por ejemplo cada 90 s) supondria ~13 veces el trafico real de telemetria". Esa
// cuenta se hizo cuando los lotes eran cada 10-20 minutos. MQTT_BATCH_SECONDS vale
// ahora 60, y la cifra se equivoca en cuatro ordenes de magnitud:
//
//   PINGREQ + PINGRESP = 4 bytes, cada 120 s       ->    120 B/hora
//   un mensaje de telemetria ~1100 bytes, cada 60 s -> 66000 B/hora
//
//   los ping son el 0,18% de la telemetria - aproximadamente 1 MB al ANO
//
// El valor viejo servia para la SUBIDA, donde una sesion muerta se descubre en el
// siguiente envio y se reconstruye en ese momento. Es incorrecto para la BAJADA de
// ordenes. Con un keepalive de 3600 s ningun extremo cuestiona una conexion TCP caida
// en silencio durante hasta una hora: Modulo B cree que esta conectado, el servidor
// cree que el cliente esta vivo, y todas las ordenes de mientras no van a ninguna
// parte. Nada en los logs parece ir mal.
//
// A 120 s el propio modem manda el PINGREQ (sin trafico AT nuestro - lo gestiona el
// modulo a partir de este ajuste), asi que un enlace muerto se detecta en un par de
// minutos y se reconstruye, y la sesion se mantiene a traves de los tiempos de NAT del
// operador aunque los nodos no tengan nada que decir.
#define MQTT_KEEPALIVE_S 120UL

// El JSON es mucho mas largo que la antigua trama binaria (~15 campos por lectura
// como texto "clave":valor pasan de 100 bytes por lectura, frente a los 30 fijos del
// formato binario): dimensionado con generosidad.
#define MQTT_PAYLOAD_MAX_BYTES 4096

// ---------- Lotes de la via UDP antigua (HISTORICO) ----------
// La subida de la via UDP era una trama binaria compacta (ver nbiot/NbiotProtocol.h),
// no CSV de texto: 10 lecturas completas de 13 campos como texto ocupan ~700-800
// bytes. Solo para la via UDP retirada (ModemNBIoT) y su prueba unitaria; la via MQTT
// usa MQTT_BATCH_* de arriba.
// EXCEPCION: NBIOT_RING_CAPACITY SI se usa hoy - es el tamano del anillo de lecturas
// pendientes de la via MQTT.
#define NBIOT_BATCH_DEFAULT_READINGS 10
#define NBIOT_BATCH_DEFAULT_SECONDS (20UL * 60UL)
#define NBIOT_RING_CAPACITY 24  // > lote maximo, para que una subida lenta no obligue a tirar lecturas en el acto
#define NBIOT_PAYLOAD_MAX_BYTES 500

// Techo que usa el static_assert de la trama binaria en NbiotProtocol.h (HISTORICO,
// solo de la via UDP).
#define NBIOT_CFG_READINGS_MAX 16  // (500 - kBatchHeaderSize) / kBatchRecordSize, ver el static_assert

// ---------- Plazos de cada estado (ms) ----------
#define NBIOT_TIMEOUT_POWERING_MS 3000
// Tiempo que se le da al modem para contestar a un AT tras soltar PWRKEY.
//
// SUBIDO de 8000 a 30000 (2026-09-11). El plazo viejo solo le daba al BC660K-GL
// ~9,7 s desde soltar PWRKEY hasta su primera respuesta antes de que el firmware lo
// diera por fallido y LE CORTARA LA ALIMENTACION: PWRKEY se suelta a t=1300 ms,
// POWERING pasa el testigo a t=3000 ms, y WAIT_AT se rendia 8 s despues. Un
// BC660K-GL necesita a menudo mas que eso para tener el UART listo, asi que la
// recuperacion apagaba un modem que todavia estaba arrancando - para siempre, porque
// cada reintento repetia la misma carrera. Sintoma observado: "-> AT" repitiendose
// sin fin sin un solo "<- OK".
//
// Tiene que seguir siendo holgadamente mayor que el peor arranque del modulo. No
// cuesta nada con el modem sano, porque WAIT_AT sale en cuanto llega el primer OK: es
// un techo, no una espera.
#define NBIOT_TIMEOUT_WAIT_AT_MS 30000
#define NBIOT_TIMEOUT_CONFIG_MS 8000
#define NBIOT_TIMEOUT_ATTACH_MS 60000  // preguntando AT+CEREG? hasta estado 1 o 5
#define NBIOT_TIMEOUT_SOCKET_MS 15000  // cubre el OK y el aviso asincrono posterior
#define NBIOT_TIMEOUT_SEND_MS 10000
#define NBIOT_AT_CMD_TIMEOUT_MS 5000  // techo de un comando simple (AT, ATE0, CPIN?, ...)
#define NBIOT_ATTACH_POLL_INTERVAL_MS 2000  // cada cuanto se repite AT+CEREG? mientras se registra
#define NBIOT_AT_PING_GAP_MS 1000  // cada cuanto se repite el "AT" de comprobacion en WAIT_AT
#define NBIOT_CSQ_POLL_INTERVAL_MS 30000  // cada cuanto se refresca el RSSI para el panel en IDLE

// ---------- Recuperacion escalonada / espera creciente ----------
// reintentar el envio -> reconectar MQTT -> volver a registrarse -> apagar y
// encender. Apagar y encender en frio obliga a un registro completo en la red (lento,
// puede ser de decenas de segundos, y lo mas caro que hace este modem), asi que solo
// puede ser el ultimo recurso, nunca la primera respuesta a un envio fallido.
#define NBIOT_MAX_SEND_RETRIES 2
#define NBIOT_MAX_SOCKET_RETRIES 2
#define NBIOT_MAX_ATTACH_RETRIES 2
#define NBIOT_BACKOFF_BASE_MS 2000
#define NBIOT_BACKOFF_MAX_MS 120000
// Tiempo que EN se mantiene desactivado antes de volver a activarlo en un apagado y
// encendido. SUBIDO (2026-09-11): 500 ms no bastan para que el rail propio del modulo
// caiga de verdad, asi que el "apagado" podia dejarlo medio alimentado en un estado
// indefinido en vez del arranque en frio limpio que se pretendia.
#define NBIOT_POWER_OFF_SETTLE_MS 4000
