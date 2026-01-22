import cv2
import numpy as np
import os
import argparse

# --- CONFIGURACIÓN ---
PIXELES_ARUCO = 300    # Resolución de un solo marcador

# Anchos válidos en MVSIM (solo valores que dan repeticiones enteras)
# Fórmula: repeticiones = 1 / ancho_mvsim
ANCHOS_VALIDOS = {
    '1.00m': 1,   # 1.00m → 1/1.00 = 1 repetición
    '0.50m': 2,   # 0.50m → 1/0.50 = 2 repeticiones
    '0.33m': 3,   # 0.33m → 1/0.33 = 3 repeticiones
    '0.25m': 4,   # 0.25m → 1/0.25 = 4 repeticiones
    '0.20m': 5,   # 0.20m → 1/0.20 = 5 repeticiones
    '0.17m': 6,   # 0.17m → 1/0.17 = 6 repeticiones (≈0.166m)
    '0.14m': 7,   # 0.14m → 1/0.14 = 7 repeticiones (≈0.143m)
    '0.13m': 8,   # 0.13m → 1/0.13 = 8 repeticiones (≈0.125m)
    '0.11m': 9,   # 0.11m → 1/0.11 = 9 repeticiones (≈0.111m)
    '0.10m': 10,  # 0.10m → 1/0.10 = 10 repeticiones
}
# ---------------------

def main():
    parser = argparse.ArgumentParser(
        description='Generador de marcadores ArUco para MVSIM',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Anchos válidos en MVSIM (repeticiones = 1 / ancho):
{chr(10).join(f'  {ancho:6s} → {reps:2d}x{reps} mosaico (textura de {reps*300}x{reps*300}px)' for ancho, reps in ANCHOS_VALIDOS.items())}

Ejemplos:
  python3 aruco_generator.py --id 0 --ancho 0.33m
  python3 aruco_generator.py --range 0 5 --ancho 0.20m
        """
    )
    
    parser.add_argument('--id', type=int, help='ID específico del marcador a generar')
    parser.add_argument('--range', type=int, nargs=2, metavar=('START', 'END'),
                        help='Generar rango de IDs (ejemplo: --range 0 10)')
    parser.add_argument('--ancho', type=str, default='0.33m',
                        choices=list(ANCHOS_VALIDOS.keys()),
                        help='Ancho del código ArUco en MVSIM (default: 0.33m → 3x3)')
    
    args = parser.parse_args()
    
    # Calcular repeticiones según el ancho seleccionado
    repeticiones = ANCHOS_VALIDOS[args.ancho]
    
    # Obtener diccionario ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    
    # Determinar qué IDs generar
    if args.range:
        ids = list(range(args.range[0], args.range[1] + 1))
    elif args.id is not None:
        ids = [args.id]
    else:
        # Modo interactivo
        print("=" * 60)
        print("GENERADOR DE MARCADORES ARUCO PARA MVSIM")
        print("=" * 60)
        print("\nAnchos válidos en MVSIM:")
        for ancho, reps in ANCHOS_VALIDOS.items():
            print(f"  {ancho}: {reps}x{reps} mosaico ({reps*PIXELES_ARUCO}x{reps*PIXELES_ARUCO}px)")
        
        print(f"\n✓ Ancho seleccionado: {args.ancho} → {repeticiones}x{repeticiones} repeticiones")
        id_input = input("\nIntroduce el ID del marcador (default: 0): ").strip()
        ids = [int(id_input) if id_input else 0]
    
    print(f"\n📌 Ancho MVSIM: {args.ancho}")
    print(f"📌 Mosaico: {repeticiones}x{repeticiones} repeticiones ({repeticiones*PIXELES_ARUCO}x{repeticiones*PIXELES_ARUCO}px)")
    print(f"📌 Generando IDs: {ids}\n")
    
    # Generar marcadores
    for id_marcador in ids:
        # Generar un solo marcador (blanco y negro)
        img_base = cv2.aruco.drawMarker(aruco_dict, id_marcador, PIXELES_ARUCO, 1)

        # Crear el lienzo gigante (mosaico)
        ancho_total = PIXELES_ARUCO * repeticiones
        alto_total = PIXELES_ARUCO * repeticiones
        
        mosaico = np.zeros((alto_total, ancho_total), dtype=np.uint8)

        # Rellenar el mosaico con repeticiones del marcador
        for fila in range(repeticiones):
            for col in range(repeticiones):
                y1 = fila * PIXELES_ARUCO
                y2 = y1 + PIXELES_ARUCO
                x1 = col * PIXELES_ARUCO
                x2 = x1 + PIXELES_ARUCO
                
                mosaico[y1:y2, x1:x2] = img_base

        # Guardar imagen
        if not os.path.exists("../aruco_textures"):
            os.makedirs("../aruco_textures")
        
        nombre = f"../aruco_textures/aruco_texture_id{id_marcador}.png"
        cv2.imwrite(nombre, mosaico)
        print(f"✅ ID {id_marcador}: {nombre}")
    
    print(f"\n✅ Generación completada: {len(ids)} marcador(es)")

if __name__ == "__main__":
    main()