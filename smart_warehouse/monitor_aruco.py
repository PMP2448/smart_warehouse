#!/usr/bin/env python3
# monitor_aruco.py
# Nodo ROS2 para monitorizar la posición del robot respecto a un marcador ArUco
# Se monitoriza Distancia (Z), Desviación Lateral (X) en el sistema del ArUco y Rotación (Yaw)

# Sistema de coordenadas ArUco:
# - El eje Z apunta hacia fuera del marcador (distancia)
# - El eje X apunta a la derecha del marcador
# - El eje Y apunta hacia arriba (no se usa en navegación horizontal)

# Sistema de coordenadas del robot (base_link):
# - El eje X apunta hacia delante del robot
# - El eje Y apunta a la izquierda del robot
# - El eje Z apunta hacia arriba del robot

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
import math
import sys

class TFMonitor(Node):
    def __init__(self):
        super().__init__('tf_monitor')

        # 1. PREPARAR EL SISTEMA TF
        # Buffer: Almacena los últimos segundos de datos de posición
        self.tf_buffer = Buffer()
        # Listener: Escucha silenciosamente las TFs y llena el buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer: Consultamos la posición 10 veces por segundo
        self.timer = self.create_timer(0.1, self.on_timer)

        print("\n" * 2)
        print("="*60)
        print("   MONITOR TF: aruco_detectado -> base_link")
        print("="*60)
        print("Esperando a que aparezca el marco 'aruco_detectado' en el árbol TF...")

    def quaternion_to_yaw(self, q):
        """Extrae la rotación del robot alrededor del eje Y (vertical).
        No depende de la posición, solo de la orientación relativa."""
        yaw = math.atan2(2.0 * (q.w * q.z - q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

    def on_timer(self):
        # TRANSFORMACIÓN: Posición del ROBOT respecto al ARUCO
        # Queremos saber el error de posición y orientación del robot
        # en el sistema de coordenadas del ArUco
        from_frame = 'aruco_detectado'  # Sistema de referencia: ArUco
        to_frame   = 'base_link'         # Lo que medimos: Robot

        try:
            # Lookup: Dónde está el robot EN EL SISTEMA del ArUco
            t = self.tf_buffer.lookup_transform(
                from_frame,  # Referencia: ArUco
                to_frame,    # Medición: Robot
                rclpy.time.Time())

            # EXTRAER ERRORES DE POSICIÓN en el sistema del ArUco:
            # X = Error lateral (+ derecha, - izquierda del código)
            # Z = Error frontal (distancia al código, + alejándose)
            # Y = Altura (no la usamos para navegación horizontal)
            
            error_x = t.transform.translation.x  # Lateral
            error_z = t.transform.translation.z  # Profundidad/Distancia
            
            # Rotación en el plano horizontal (XZ)
            yaw_rad = self.quaternion_to_yaw(t.transform.rotation)
            yaw_deg = math.degrees(yaw_rad)

            # IMPRIMIR errores de posición y orientación
            sys.stdout.write("\033[K") 
            print(f"\r📍 Error Robot vs ArUco | X (lateral): {error_x:+6.3f}m | Z (profundidad): {error_z:6.3f}m | Rotación: {yaw_deg:+6.1f}°", end="")

        except TransformException as ex:
            # Si el ArUco no se ve, el TF desaparece y entra aquí
            sys.stdout.write("\033[K")
            print(f"\r⏳ Buscando transformación... (ArUco no visible)", end="")
            return

def main():
    rclpy.init()
    node = TFMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nMonitor finalizado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()