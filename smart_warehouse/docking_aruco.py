import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException
import math
import time
import sys

# --- CONSTANTES DE ESTADO ---
ST_SEARCHING         = 0 # Buscando ArUco
ST_ALIGNING_ANGLE       = 1 # Alineando con 30º respecto a carril de aproximación
ST_ALIGNING_LAT      = 2 # Alineando lateralmente con carril de aproximación
ST_ALIGNING_0        = 3 # Alineando orientación a 0º respecto al carril
ST_APPROACHING       = 4 # Aproximación inicial
ST_APPROACHING_FINAL = 5 # Aproximándose a pose final
ST_FINISHED          = 6 # Aproximación finalizada

# 90º -> mirando al ArUco
# 0º -> perpendicular al ArUco (derecha)
# -90º -> mirando en sentido contrario al ArUco
# Usamos radianes para sumar a la medida (no grados)
yaw_offset = math.radians(-90.0)  # 0 rad al mirar hacia el ArUco

class ArucoDocking(Node):
    def __init__(self):
        super().__init__('aruco_docking')

        # 1. PARÁMETROS (Ajustables desde terminal)
        self.declare_parameter('target_dist', 0.5)     # Distancia final (m)
        self.declare_parameter('pre_target_dist', 1.5) # Lateral final (m)
        self.declare_parameter('tolerance_dist', 0.02) # 2cm de margen en distancia
        self.declare_parameter('tolerance_lat', 0.04)  # 4cm de margen lateral
        self.declare_parameter('tolerance_yaw', 3)     # ~3 grados de margen
        self.declare_parameter('max_v', 0.2)           # Max vel lineal 
        self.declare_parameter('max_w', 4.0)           # Max vel angular (float)
        self.declare_parameter('k_v', 0.5)  # Velocidad de aproximación final
        self.declare_parameter('k_w', 0.4)  # Velocidad angular de alineamiento
        self.declare_parameter('aligning_angle', 45)  # Hz del loop de control

        self.target_z = self.get_parameter('target_dist').value
        self.pre_target_z = self.get_parameter('pre_target_dist').value
        self.tolerance_dist = self.get_parameter('tolerance_dist').value
        self.tol_lat  = self.get_parameter('tolerance_lat').value
        self.tol_yaw  = self.get_parameter('tolerance_yaw').value
        self.k_v = self.get_parameter('k_v').value
        self.k_w = self.get_parameter('k_w').value
        self.aligning_angle = 30  # Grados para alineamiento inicial

        # 2. CONFIGURACIÓN TF (LECTURA)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. CONTROL (ESCRITURA)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 4. LOOP DE CONTROL (TIMER 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Variables internas
        self.state = ST_SEARCHING
        self.last_aruco_time = 0
        self.get_logger().info("Docking Node Iniciado. Freq: 20Hz")

    def quaternion_to_yaw(self, q):
        """Extrae la rotación del robot alrededor del eje Y (vertical).
        No depende de la posición, solo de la orientación relativa."""
        yaw = math.atan2(2.0 * (q.w * q.z - q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

    def get_aruco_tf(self):
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
            yaw_rad = self.quaternion_to_yaw(t.transform.rotation) + yaw_offset
            yaw_deg = math.degrees(yaw_rad)

            # IMPRIMIR errores de posición y orientación
            sys.stdout.write("\033[K") 
            print(f"\r📍 Error Robot vs ArUco | X (lateral): {error_x:+6.3f}m | Z (profundidad): {error_z:6.3f}m | Rotación: {yaw_deg:+6.1f}°", end="")
            return error_x, error_z, yaw_rad

        except TransformException as ex:
            # Si el ArUco no se ve, el TF desaparece y entra aquí
            sys.stdout.write("\033[K")
            print(f"\r⏳ Buscando transformación... (ArUco no visible)", end="")
            return

    def control_loop(self):
        # 1. LEER DATOS
        data = self.get_aruco_tf()
        cmd = Twist()

        # MAQUINA DE ESTADOS
        if data is None:
            # Si perdemos el ArUco, paramos y volvemos a buscar
            if self.state != ST_SEARCHING and self.state != ST_FINISHED:
                self.get_logger().warn("ArUco perdido. Esperando...")
                self.state = ST_SEARCHING
            # Velocidad 0 por seguridad
            self.cmd_pub.publish(Twist())
            return

        # Desempaquetar datos validos
        # x = Error Lateral (queremos 0)
        # z = Distancia (queremos target_z)
        # yaw = Error Angulo (queremos 0, asumiendo robot y aruco alineados)
        rx, rz, ryaw = data 

        # --- LÓGICA DE ESTADOS ---
        
        if self.state == ST_SEARCHING:
            self.get_logger().info("ArUco Encontrado. Iniciando Alineamiento.")
            self.state = ST_ALIGNING_ANGLE

        elif self.state == ST_ALIGNING_ANGLE:
            # FASE 1: Alineamiento inicial con +-30º respecto al carril
            # Si el robot se situa a la izquierda del carril, debe alinear -30º
            # Si el robot se situa a la derecha del carril, debe alinear +30º
            if rx < 0:
                target_yaw = math.radians(-self.aligning_angle)
            else:
                target_yaw = math.radians(self.aligning_angle)
            
            yaw_error = ryaw - target_yaw
            w = -self.k_w * yaw_error # Control Proporcional

            if abs(yaw_error) < math.radians(self.tol_yaw):
                self.get_logger().info("Alineamiento 30º completado.")
                self.state = ST_ALIGNING_LAT
                w = 0.0

            cmd.angular.z = w
            cmd.linear.x = 0.0

        elif self.state == ST_ALIGNING_LAT:
            # FASE 2: Alineamiento lateral con carril de aproximación
            lat_error = rx
            v = abs(self.k_v * lat_error)  # Control Proporcional

            if abs(lat_error) < self.tol_lat:
                self.get_logger().info("Alineamiento lateral completado.")
                self.state = ST_ALIGNING_0
                v = 0.0

            cmd.angular.z = 0.0
            cmd.linear.x = v

        elif self.state == ST_ALIGNING_0:
            # FASE 3: Alineamiento angular a 0º respecto al carril
            yaw_error = ryaw
            w = -self.k_w * yaw_error  # Control Proporcional

            if abs(yaw_error) < math.radians(self.tol_yaw):
                self.get_logger().info("Alineamiento 0º completado.")
                self.state = ST_APPROACHING
                w = 0.0

            cmd.angular.z = w
            cmd.linear.x = 0.0

        elif self.state == ST_APPROACHING:
            # FASE 4: Aproximación inicial hasta pre_target_z
            dist_error = rz - self.pre_target_z
            v = self.k_v * dist_error  # Control Proporcional

            if abs(dist_error) < self.tolerance_dist:
                self.get_logger().info("Aproximación inicial completada.")
                self.state = ST_APPROACHING_FINAL
                v = 0.0

            cmd.linear.x = v
            cmd.angular.z = 0.0
        
        elif self.state == ST_APPROACHING_FINAL:
            # FASE 5: Aproximación final hasta target_z
            dist_error = rz - self.target_z
            v = self.k_v * dist_error  # Control Proporcional

            if abs(dist_error) < self.tolerance_dist:
                self.get_logger().info("Aproximación final completada. Docking terminado.")
                self.state = ST_FINISHED
                v = 0.0

            cmd.linear.x = v
            cmd.angular.z = 0.0

        elif self.state == ST_FINISHED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # Aquí podrías reiniciar si el robot se mueve

        # 3. SEGURIDAD Y PUBLICACIÓN
        # Saturación (Clamp) para no correr demasiado
        max_v = self.get_parameter('max_v').value
        max_w = self.get_parameter('max_w').value
        
        # Asegurar tipos float para mensajes ROS (evitar ints)
        cmd.linear.x = float(max(min(cmd.linear.x, max_v), -max_v))
        cmd.angular.z = float(max(min(cmd.angular.z, max_w), -max_w))

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDocking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # Parada final
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()