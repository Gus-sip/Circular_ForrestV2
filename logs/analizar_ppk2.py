# -*- coding: utf-8 -*-
"""Analiza una exportacion CSV del Nordic Power Profiler contra el calendario del nodo.

Por que existe: el PPK2 da una corriente cada microsegundo y nada mas. Lo que hace
falta para el presupuesto energetico es cuanto consume CADA FASE, y eso exige
separar el suelo de los picos y saber que trabajo causo cada pico.

LO QUE LO HACE POSIBLE es que desde el 2026-10-09 el calendario del nodo esta
ALINEADO AL RELOJ: un trabajo de periodo P cae en los multiplos de P de la hora
real, medido con un error del 0,1%. Asi que cada pico se puede nombrar por el
minuto en el que cae, en vez de adivinarlo por su forma.

    NORMAL      :00 y :30 barrido completo | :01 y :31 envio | :20 y :40 VOC
                resto de multiplos de 5: T/H/P + viento
    PRE-ALARMA  :00 y :30 todo + CO2 | cada 15 envio | cada 10 VOC+CO+particulas
                cada 5 viento | cada 3 T/H/P
    ALARMA      cada 1 T/H/P | cada 5 VOC+CO+particulas+viento | cada 3 envio
                CO2 sigue en 30

Uso:
    python logs/analizar_ppk2.py captura.csv --estado prealarma
    python logs/analizar_ppk2.py captura.csv --estado normal --inicio "2026-10-10 20:15:00"

El CSV del Power Profiler trae una columna de tiempo en microsegundos desde el
inicio de la grabacion y otra de corriente en microamperios. Si se le pasa
--inicio con la hora de pared en que se empezo a grabar, el script sitúa cada
muestra en el reloj y puede nombrar los picos; sin ella hace el analisis por
umbral, que no necesita saber la hora.
"""
import argparse
import csv
import statistics as st
import sys

# Periodos en minutos por estado, de notes/estrategia_lectura_por_tiempo.md
CALENDARIO = {
    'normal':    {'T/H/P': 5,  'VOC': 20, 'CO2': 30, 'CO': 30, 'PM': 30, 'viento': 5,  'envio': 30},
    'prealarma': {'T/H/P': 3,  'VOC': 10, 'CO2': 30, 'CO': 10, 'PM': 10, 'viento': 5,  'envio': 15},
    'alarma':    {'T/H/P': 1,  'VOC': 5,  'CO2': 30, 'CO': 5,  'PM': 5,  'viento': 5,  'envio': 3},
}


def leer_csv(ruta):
    """Devuelve [(segundos_desde_inicio, microamperios)]. Tolera varios formatos."""
    filas = []
    with open(ruta, newline='', encoding='utf-8', errors='replace') as f:
        lector = csv.reader(f)
        cabecera = next(lector, None)
        # El Power Profiler exporta "Timestamp(ms),Current(uA)" o similar; se buscan
        # las dos primeras columnas numericas y se deduce la unidad por el nombre.
        divisor = 1000.0 if cabecera and 'ms' in (cabecera[0] or '').lower() else 1e6
        for fila in lector:
            if len(fila) < 2:
                continue
            try:
                filas.append((float(fila[0]) / divisor, float(fila[1])))
            except ValueError:
                continue
    return filas


def resumen(muestras, umbral_ua):
    """Separa reposo de actividad por umbral y devuelve las cifras del presupuesto."""
    corrientes = [c for _, c in muestras]
    if not corrientes:
        return None
    reposo = [c for c in corrientes if c < umbral_ua]
    activo = [c for c in corrientes if c >= umbral_ua]

    dur = muestras[-1][0] - muestras[0][0]
    media = st.mean(corrientes)
    # Carga en mAh: media en uA por las horas, pasado a mA
    mah = media / 1000.0 * (dur / 3600.0)
    return {
        'duracion_s': dur,
        'n': len(corrientes),
        'media_ua': media,
        'mediana_ua': st.median(corrientes),
        'max_ua': max(corrientes),
        'reposo_ua': st.median(reposo) if reposo else 0.0,
        'pct_activo': 100.0 * len(activo) / len(corrientes),
        'mah_captura': mah,
        'mah_hora': media / 1000.0,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv')
    p.add_argument('--estado', choices=sorted(CALENDARIO), default='normal')
    p.add_argument('--umbral', type=float, default=None,
                   help='uA por encima de los cuales se considera actividad; '
                        'por defecto, el doble de la mediana')
    p.add_argument('--pack-mah', type=float, default=10000.0,
                   help='capacidad del pack para estimar autonomia (10 Ah por defecto)')
    a = p.parse_args()

    muestras = leer_csv(a.csv)
    if not muestras:
        print('No se pudo leer ninguna muestra de %s' % a.csv)
        print('Esperaba dos columnas: tiempo y corriente.')
        sys.exit(1)

    umbral = a.umbral if a.umbral else 2.0 * st.median([c for _, c in muestras])
    r = resumen(muestras, umbral)

    print('=' * 68)
    print('CONSUMO EN ESTADO %s' % a.estado.upper())
    print('=' * 68)
    print('  muestras          %d' % r['n'])
    print('  duracion          %.1f s  (%.2f h)' % (r['duracion_s'], r['duracion_s'] / 3600))
    print('  umbral actividad  %.0f uA' % umbral)
    print()
    print('  CORRIENTE MEDIA   %.3f mA      <- la cifra del presupuesto' % (r['media_ua'] / 1000))
    print('  mediana           %.3f mA' % (r['mediana_ua'] / 1000))
    print('  reposo            %.3f mA' % (r['reposo_ua'] / 1000))
    print('  pico maximo       %.1f mA' % (r['max_ua'] / 1000))
    print('  tiempo activo     %.1f%%' % r['pct_activo'])
    print()
    print('  consumo/hora      %.3f mAh' % r['mah_hora'])
    print('  consumo/dia       %.1f mAh' % (r['mah_hora'] * 24))
    dias = a.pack_mah / (r['mah_hora'] * 24) if r['mah_hora'] else 0
    print('  AUTONOMIA         %.1f dias con un pack de %.0f mAh' % (dias, a.pack_mah))
    print()
    print('  Ojo: la autonomia supone este estado las 24 h. Un nodo real pasa casi')
    print('  todo el tiempo en NORMAL, asi que la cifra util es la de ese estado;')
    print('  las de prealarma y alarma dicen cuanto cuesta un episodio.')
    print()
    print('CALENDARIO DE %s, para nombrar los picos:' % a.estado.upper())
    for k, v in sorted(CALENDARIO[a.estado].items(), key=lambda x: x[1]):
        print('  cada %2d min   %s' % (v, k))


if __name__ == '__main__':
    main()
