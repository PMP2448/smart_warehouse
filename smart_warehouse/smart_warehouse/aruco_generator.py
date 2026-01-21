

import cv2
import numpy as np
import os
import argparse
import sys


# Usamos el diccionario ORIGINAL (el más antiguo y compatible)
DICCIONARIO = cv2.aruco.DICT_ARUCO_ORIGINAL
PIXELES_ARUCO = 300    # Tamaño en píxeles de cada marcador
REPETICIONES = 5       # Para mosaico MVSIM



def get_pkg_dir():
    # Ruta absoluta fija al paquete fuente
    return '/home/pmp/ros2_ws/src/smart_warehouse'

def generar_mosaico_y_guardar(aruco_dict, id_marcador, pixeles_aruco, repeticiones, carpeta_destino):
    # 1. Generar marcador base
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img_base = cv2.aruco.generateImageMarker(aruco_dict, id_marcador, pixeles_aruco, 1)
    else:
        img_base = cv2.aruco.drawMarker(aruco_dict, id_marcador, pixeles_aruco, 1)

    # 2. Crear el mosaico (repeticiones x repeticiones)
    ancho_total = pixeles_aruco * repeticiones
    alto_total = pixeles_aruco * repeticiones
    mosaico = np.zeros((alto_total, ancho_total), dtype=np.uint8)

    for fila in range(repeticiones):
        for col in range(repeticiones):
            y1, y2 = fila * pixeles_aruco, (fila + 1) * pixeles_aruco
            x1, x2 = col * pixeles_aruco, (col + 1) * pixeles_aruco
            mosaico[y1:y2, x1:x2] = img_base

    # 3. Flip vertical para MVSIM
    mosaico = cv2.flip(mosaico, 0)

    # 4. Guardar en carpeta dentro del paquete
    pkg_dir = get_pkg_dir()
    destino_absoluto = os.path.join(pkg_dir, carpeta_destino)
    if not os.path.exists(destino_absoluto):
        os.makedirs(destino_absoluto)
    nombre = os.path.join(destino_absoluto, f"aruco_texture_id{id_marcador}.png")
    cv2.imwrite(nombre, mosaico)
    print(f"✅ LISTO. Textura guardada en: {nombre}")
    print("⚠️  IMPORTANTE: CIERRA Y VUELVE A ABRIR MVSIM PARA VER EL CAMBIO.")


def guardar_marcador_individual(aruco_dict, id_marcador, pixeles_aruco, carpeta_destino):
    # Genera y guarda un solo marcador para impresión
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(aruco_dict, id_marcador, pixeles_aruco, 1)
    else:
        img = cv2.aruco.drawMarker(aruco_dict, id_marcador, pixeles_aruco, 1)
    pkg_dir = get_pkg_dir()
    destino_absoluto = os.path.join(pkg_dir, carpeta_destino)
    if not os.path.exists(destino_absoluto):
        os.makedirs(destino_absoluto)
    nombre = os.path.join(destino_absoluto, f"aruco_marker_id{id_marcador}.png")
    cv2.imwrite(nombre, img)
    print(f"🖨️  Marcador individual guardado en: {nombre}")

def main():
    parser = argparse.ArgumentParser(description="Generador de marcadores ArUco para MVSIM y para impresión.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--id', type=int, help='ID único del marcador a generar')
    group.add_argument('--range', nargs=2, type=int, metavar=('INICIO', 'FIN'), help='Rango de IDs a generar (INICIO FIN, ambos inclusive)')
    parser.add_argument('--print', action='store_true', help='Guardar también los marcadores individuales en aruco_markers para impresión')
    parser.add_argument('--size', type=int, default=PIXELES_ARUCO, help='Tamaño en píxeles del marcador (por defecto 300)')
    parser.add_argument('--reps', type=int, default=REPETICIONES, help='Repeticiones para mosaico MVSIM (por defecto 5)')
    args = parser.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICCIONARIO)

    ids = []
    if args.id is not None:
        ids = [args.id]
    elif args.range is not None:
        inicio, fin = args.range
        ids = list(range(inicio, fin + 1))

    for id_marcador in ids:
        print(f"🛠️  Generando marcador ID {id_marcador}...")
        generar_mosaico_y_guardar(aruco_dict, id_marcador, args.size, args.reps, "aruco_textures")
        if args.print:
            guardar_marcador_individual(aruco_dict, id_marcador, args.size, "aruco_markers")

if __name__ == "__main__":
    main()