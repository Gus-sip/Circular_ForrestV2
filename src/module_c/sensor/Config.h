#pragma once

// =============================================================================
// CONFIGURACION DE LOS SENSORES DE MODULO C
// =============================================================================
//
// Nada de numeros magicos dentro de los drivers: todo lo ajustable esta aqui.

// Direcciones I2C de los sensores, confirmadas con escaneos I2C y lecturas que
// funcionan. Los pines que fijan la direccion quedan al aire en mas de una de
// estas placas, asi que otra unidad podria necesitar otro valor con toda
// legitimidad; por eso cada driver recibe su direccion como argumento en vez de
// llevarla fija dentro.
#define BME690_I2C_ADDR 0x76   // pin SDO; la alternativa 0x77 se prueba sola si falla esta
#define SEN0466_I2C_ADDR 0x74
#define BMV080_I2C_ADDR 0x57   // CS=alto, SDO=alto. Confirmado en la placa fabricada por
                               // escaneo I2C el 2026-09-04: 6 de 6 arranques con ACK en 0x57,
                               // junto a 0x74 (SEN0466) y 0x77 (BME690). Un escaneo anterior -
                               // el primero tras tener los rails apagados - mostro 0x56; tomar
                               // un 0x56 como el sensor pillado a medio arrancar, no como un
                               // cambio de direccion. (El driver prueba igualmente 0x54-0x57.)

// Tiempos de calentamiento/arranque. Los marcados SIN CONFIRMAR no salen de una
// pagina de hoja de datos: son valores prudentes que funcionaron en la practica,
// dejados como constantes con nombre para encontrarlos y corregirlos facilmente.
#define CM1106_WARMUP_MS 3000          // SIN CONFIRMAR frente a la hoja de datos
#define BMV080_STARTUP_DELAY_MS 5000   // SIN CONFIRMAR - arranque, precalentamiento del laser y autotest

// Velocidades de los UART
#define CM1106_BAUD 9600
#define CALYPSO_BAUD 38400

// Cada cuanto imprimia una tabla de lecturas un antiguo programa de banco. Hoy no
// lo usa ningun programa del proyecto.
// [ELIMINAR] sin uso: del firmware antiguo sin calendario.
#define SAMPLE_INTERVAL_MS 1000
