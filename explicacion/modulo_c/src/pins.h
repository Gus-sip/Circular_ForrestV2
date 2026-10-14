#pragma once

// =============================================================================
// MAPA DE PINES DE MODULO C (PLACA CHIP FOREST, ESP32-S3)
// =============================================================================
//
// Este archivo empezo describiendo un banco de pruebas sin placa fabricada (un
// rail de 3V3 siempre encendido, sin transistores de corte). Hoy describe la placa
// real. Cada valor de abajo se confirmo en la practica (escaneos I2C, lecturas que
// funcionan), no se dio por bueno a partir de un esquema.
//
// Resumen:
//   GPIO1/2   I2C (BME690, BMV080, SEN0466)      GPIO10  rail 3V3   (LOW  = on)
//   GPIO3     CM1106 EN                           GPIO11  rail 5V    (HIGH = on)
//   GPIO4/5   UART radio LoRa                     GPIO13  rail LoRa  (LOW  = on)
//   GPIO6/7   UART CM1106                         GPIO14  medida de bateria (analogica)
//   GPIO8/9   UART Calypso                        GPIO15  BMV080 EN  (LOW  = on)
//   GPIO12    CM1106 RDY (NO TOCAR)               GPIO16  SEN0466 EN (LOW  = on)
//                                                 GPIO21  LED de estado

// Bus I2C compartido: BME690, SEN0466 y BMV080 cuelgan todos de el.
#define PIN_I2C_SDA 1
#define PIN_I2C_SCL 2

// Sensor de CO2 Cubic CM1106SL-NS: UART2. Cuelga del rail de 5 V conmutado.
#define PIN_CM1106_TX 6  // TX del ESP -> RX del sensor
#define PIN_CM1106_RX 7  // RX del ESP <- TX del sensor
#define PIN_CM1106_EN 3  // activo a nivel alto, con una resistencia de 1k en serie en la placa

// Anemometro Calypso ULP PRO: UART1. Variante TTL-UART/NMEA confirmada (no RS485).
// 2026-09-01: RX paso de 9 a 8 y TX de 8 a 9. El TX del sensor (cable verde) esta
// fisicamente en GPIO8: un barrido en banco por GPIO5/6/8/9 y velocidades de 4800 a
// 115200 solo obtuvo una frase $IIMWV valida con RX=GPIO8 a 38400. Ademas esta
// unidad necesita 5 V, no 3,3 V: a 3,3 V estaba muda y empezo a emitir en cuanto se
// paso a 5 V.
#define PIN_CALYPSO_TX 9  // TX del ESP -> RX/amarillo del sensor (no se usa: el sensor emite solo)
#define PIN_CALYPSO_RX 8  // RX del ESP <- TX/verde del sensor

// GPIO12 va al CM1106 de CUBIC (el sensor de CO2; Cubic es el fabricante), que es
// lo que queria decir la nota original de "futura linea RDY del CM1106".
//
// NO CONDUCIRLO. Se puso a nivel bajo un momento mientras se buscaba la
// habilitacion del BMV080, antes de que se describiera la placa. El CM1106
// funciona, y forzar una linea contra un sensor que funciona es una buena manera
// de dejar de tenerlo.
#define PIN_CM1106_RDY 12

// Habilitacion del BMV080. Tiene que estar a nivel BAJO para que el sensor funcione.
//
// Sin conducir, el BMV080 abre perfectamente por I2C - contesta en su direccion y
// bmv080_open() tiene exito - pero se niega a encender el laser y devuelve el
// estado 114 del SDK (E_BMV080_ERROR_OPERATION_MODE_CHANNELS_OUT_OF_SYNC), cuya
// pista de Bosch es "comprobar que la alimentacion del BMV080 es suficiente y
// estable". Esa pista apunta a la alimentacion, que es exactamente lo que parece
// una habilitacion sin activar vista desde el sensor.
//
// Paso de 12 a 14 y a 15 conforme se describio la placa; el 12 resulto ser el
// CM1106 y el 14 la medida de bateria. Se activa junto con los rails, antes de abrir
// ningun sensor, para que este estable cuando el SDK hable con la pieza.
#define PIN_BMV080_EN 15
#define PIN_BMV080_EN_ACTIVE LOW

// Medida de la tension de la bateria - ENTRADA ANALOGICA, NUNCA UNA SALIDA.
//
// No se debe conducir nunca. Durante la busqueda de la habilitacion del BMV080 se
// confundio GPIO14 con otra candidata y se mantuvo a nivel bajo como salida: eso
// pelea contra el divisor que la alimenta y puede danarlo. Se deja escrito aqui
// para que nadie lo repita.
//
// GPIO14 es ADC2_CH3 en el ESP32-S3. El ADC2 no esta disponible con el WiFi activo,
// lo que no importa en Modulo C (sin WiFi) pero si en Modulo B.
//
// El nombre viene de la epoca de los supercondensadores. La conversion de la
// tension del pin a porcentaje de carga esta en chip_forest_lora_tx.cpp
// (SUPERCAP_*). OJO: desde el 2026-09-24 la fuente es una bateria de litio, y esa
// conversion se calculo para supercondensadores.
#define PIN_SUPERCAP_SENSE 14

// ---------- Habilitaciones de los rails de sensores (GPIO10/11) ----------
// La placa real conecta los DOS rails de alimentacion de sensores a traves de
// estos pines: cada uno maneja un transistor que cierra el circuito hacia el rail
// de 5 V y el de 3V3 respectivamente (confirmado 2026-09-04).
//
// Hay que activarlos al principio de setup(), antes de Wire.begin() y de cualquier
// begin() de sensor: desactivados, los sensores no tienen corriente, y un escaneo
// I2C o una prueba por UART no encontraria nada y pareceria hardware muerto. El
// Calypso y el CM1106 necesitan precisamente el rail de 5 V.
//
// LAS DOS PUERTAS TIENEN POLARIDAD OPUESTA. Se establecio el 2026-09-08 probando
// las cuatro combinaciones de GPIO10/11 y leyendo el CM1106 (el unico aparato de
// 5 V conectado) en cada una:
//
//     GPIO10=LOW  GPIO11=LOW   -> mudo
//     GPIO10=LOW  GPIO11=HIGH  -> contesta     <- rail de 5 V encendido
//     GPIO10=HIGH GPIO11=LOW   -> mudo
//     GPIO10=HIGH GPIO11=HIGH  -> contesta     <- rail de 5 V encendido
//
// Asi que GPIO11 maneja el rail de 5 V y es ACTIVO A NIVEL ALTO, mientras que GPIO10
// (3V3) es activo a nivel bajo, como se supuso al principio: los sensores de 3V3
// siempre han funcionado con el a LOW.
//
// Esto estuvo oculto durante dias por la averia de la masa flotante: sin una
// referencia firme, poner GPIO11 "a LOW" no era un nivel bajo real en el transistor,
// y el rail de 5 V seguia encendido por casualidad. Al reparar la masa, LOW paso a
// ser un nivel bajo de verdad y apago el rail de 5 V como debia, lo que parecio que
// el CM1106 se moria. No se moria: estaba sin corriente.
#define PIN_PCB_EN_A 10          // rail de sensores de 3V3
#define PIN_PCB_EN_A_ACTIVE LOW
#define PIN_PCB_EN_B 11          // rail de sensores de 5V
#define PIN_PCB_EN_B_ACTIVE HIGH
#define PIN_PCB_EN_SETTLE_MS 300

// Se conserva para que compilen programas antiguos, pero solo es correcto para la
// puerta A. Mejor usar las macros por puerta de arriba.
#define PIN_PCB_EN_ACTIVE PIN_PCB_EN_A_ACTIVE

// ---------- Puerta de alimentacion de la radio LoRa RYLR998 (GPIO13) ----------
// Tercera puerta, aparte de las dos anteriores: un transistor P-FET en la
// alimentacion de la radio, LOW = encendido. Mismo pin y misma convencion que el
// LORA_EN_PIN de Modulo B (puerta Q4, encontrada probando pines el 2026-08-19): esta
// placa reutiliza ese diseno. El firmware de Modulo C no la conducia, y por eso el
// RYLR998 no contestaba a un AT a ninguna velocidad: no tenia corriente.
// Confirmado por el usuario el 2026-09-04.
// El UART de la radio es RXD=GPIO5 (TX del ESP) / TXD=GPIO4 (RX del ESP): las
// pistas se nombran desde el punto de vista del microcontrolador, asi que el cruce
// es correcto tal como esta.
#define PIN_LORA_EN 13
#define PIN_LORA_EN_ACTIVE LOW
#define PIN_LORA_EN_SETTLE_MS 200

// ---------- Puerta de alimentacion del sensor de CO SEN0466 (GPIO16) ----------
// Cuarta puerta, anadida a la placa el 2026-10-14. Hasta entonces el SEN0466 iba
// conectado directamente a la alimentacion, saltandose todos los transistores, y
// por eso ningun rail podia liberar el bus I2C cuando el sensor lo dejaba colgado.
// Con la puerta montada y GPIO16 sin conducir, el sensor NO tiene corriente y
// desaparece del bus ("SEN0466: NOT FOUND", sin ACK en 0x74): ese es el sintoma de
// olvidar este pin, no de un sensor muerto.
#define PIN_SEN0466_EN 16
#define PIN_SEN0466_EN_ACTIVE LOW

// ---------- LED de estado (GPIO21) ----------
// LED direccionable WS2812 en el modulo. Se confirmo el 21 probando todos los GPIO
// seguros: NO es el PIN_NEOPIXEL 48 de la variante esp32s3, ni el GPIO38 que usan
// algunas placas. Se maneja con Adafruit_NeoPixel en orden NEO_RGB (ver ledWrite()
// en chip_forest_lora_tx.cpp).
//
// No usar nunca RGB_BUILTIN: la variante lo define como
// SOC_GPIO_PIN_COUNT + PIN_NEOPIXEL = 97, un valor especial que digitalWrite()
// trata aparte. Las funciones de pixel quieren un GPIO real, asi que 97 apunta a un
// pin que no existe y no enciende nada, sin dar ningun error.
#define PIN_STATUS_LED 21
