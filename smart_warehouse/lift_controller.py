#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════╗
║        🏗️ LIFT CONTROLLER - Elevador Físico en Mvsim 🏗️       ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║   CONTROLES:                                                  ║
║   ┌───┐                                                       ║
║   │ L │  L = Enganchar/Soltar pallet más cercano              ║
║   └───┘                                                       ║
║   │ P │  P = Mostrar posición actual y pallets                ║
║                                                               ║
║   El pallet se elevará y seguirá al forklift en Mvsim.        ║
║                                                               ║
║   Q / ESC = Salir                                             ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

import threading
import sys
import termios
import tty
import select
import math
import time

# Cliente mvsim para mover objetos
# Añadir ruta de mvsim al path
MVSIM_PYTHON_PATH = '/opt/ros/jazzy/lib/python3/dist-packages'
if MVSIM_PYTHON_PATH not in sys.path:
    sys.path.insert(0, MVSIM_PYTHON_PATH)

try:
    from mvsim_comms import pymvsim_comms
    from mvsim_msgs import SrvSetPose_pb2
    MVSIM_AVAILABLE = True
except ImportError as e:
    MVSIM_AVAILABLE = False
    print(f"⚠️  mvsim_comms no disponible: {e}")


# ═══════════════════════════════════════════════════════════════
# CONFIGURACIÓN
# ═══════════════════════════════════════════════════════════════

# Offset del pallet cuando está enganchado
GRASP_OFFSET_X = 0.8    # metros delante del forklift (reducido para acercarlo)
GRASP_OFFSET_Z = 0.0    # altura del pallet levantado (0.0 = a ras de suelo)

# Distancia máxima para enganchar un pallet (metros)
MAX_GRASP_DISTANCE = 10.0  # Aumentado para facilitar el enganche

# Posiciones iniciales de los pallets (deben coincidir con el mundo actual: warehouse_numerated.world.xml)
PALLET_POSITIONS = {
    # shelf 1, lado +Y
    'pallet_s1p1': (6.65, 7.25),
    'pallet_s1p2': (2.00, 7.25),
    'pallet_s1p3': (-2.00, 7.25),
    'pallet_s1p4': (-6.65, 7.25),
    # shelf 1, lado -Y
    'pallet_s1p5': (-6.65, 2.90),
    'pallet_s1p6': (2.00, 2.90),
    'pallet_s1p7': (-2.00, 2.90),
    'pallet_s1p8': (6.65, 2.90),
    # shelf 2, lado -Y
    'pallet_s2p1': (6.65, -7.25),
    'pallet_s2p2': (2.00, -7.25),
    'pallet_s2p3': (-2.00, -7.25),
    'pallet_s2p4': (-6.65, -7.25),
    # shelf 2, lado +Y
    'pallet_s2p5': (-6.65, -2.90),
    'pallet_s2p6': (2.00, -2.90),
    'pallet_s2p7': (-2.00, -2.90),
    'pallet_s2p8': (6.65, -2.90),
    # shelf 3, lado +X
    'pallet_s3p1': (14.00, -6.60),
    'pallet_s3p2': (14.00, -2.15),
    'pallet_s3p3': (14.00, 2.15),
    'pallet_s3p4': (14.00, 6.60),
}

# Topics de odometría a probar (el primero que funcione)
ODOM_TOPICS = [
    '/odom',
    '/forklift/odom',
    '/forklift/base_pose_ground_truth',
    '/mvsim_node/forklift/odom',
]

class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')
        
        # Cliente mvsim
        self.client = None
        self.connected = False
        
        # Estado
        self.grasped_pallet = None
        self.is_lifted = False
        self.running = True
        
        # Posición del forklift
        self.forklift_x = 0.0
        self.forklift_y = 0.0
        self.forklift_yaw = 0.0
        self.odom_received = False
        self.odom_topic_found = None
        
        # Posiciones de pallets (actualizables)
        self.pallet_positions = dict(PALLET_POSITIONS)
        
        # Suscribirse a TODOS los topics de odometría posibles
        self.odom_subs = []
        for topic in ODOM_TOPICS:
            sub = self.create_subscription(
                Odometry,
                topic,
                lambda msg, t=topic: self.odom_callback(msg, t),
                10
            )
            self.odom_subs.append(sub)
            self.get_logger().info(f'📡 Escuchando: {topic}')
        
        # Publisher estado
        self.lift_state_pub = self.create_publisher(
            Bool,
            '/forklift/lift_state',
            10
        )

        # Subscribers for Interface Commands
        self.sub_grasp = self.create_subscription(String, 'agarre', self.grasp_callback, 10)
        self.sub_deposit = self.create_subscription(String, 'deposicion', self.deposit_callback, 10)
        
        # Timer para actualizar pallet
        self.timer = self.create_timer(0.02, self.update_pallet_position)
        
        # Conectar a mvsim
        self.connect_mvsim()
        
        self.get_logger().info(__doc__)

    def grasp_callback(self, msg):
        if "ON" in msg.data:
            self.get_logger().info("📥 Recibido comando AGARRE")
            self.grasp()

    def deposit_callback(self, msg):
        if "ON" in msg.data:
            self.get_logger().info("📤 Recibido comando DEPOSICION")
            self.release()

    def connect_mvsim(self):
        """Conecta al servidor mvsim"""
        if not MVSIM_AVAILABLE:
            self.get_logger().error('mvsim_comms no disponible')
            return False
        
        try:
            self.client = pymvsim_comms.mvsim.Client()
            self.client.setName("lift_controller")
            self.client.connect()
            self.connected = True
            self.get_logger().info('✅ Conectado a mvsim')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Error conectando a mvsim: {e}')
            self.connected = False
            return False

    def odom_callback(self, msg, topic):
        """Actualiza posición del forklift"""
        self.forklift_x = msg.pose.pose.position.x
        self.forklift_y = msg.pose.pose.position.y
        
        # Extraer yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.forklift_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.odom_received:
            self.odom_topic_found = topic
            self.get_logger().info(f'✅ Odometría recibida de: {topic}')
        
        self.odom_received = True

    def find_nearest_pallet(self):
        """Encuentra el pallet más cercano"""
        nearest = None
        min_dist = float('inf')
        
        for name, (px, py) in self.pallet_positions.items():
            dist = math.sqrt((px - self.forklift_x)**2 + (py - self.forklift_y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = name
        
        return nearest, min_dist

    def show_positions(self):
        """Muestra la posición actual del robot y los pallets"""
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('📍 POSICIONES ACTUALES')
        self.get_logger().info('═' * 50)
        
        if self.odom_received:
            self.get_logger().info(f'🚜 Forklift: ({self.forklift_x:.2f}, {self.forklift_y:.2f}) yaw: {math.degrees(self.forklift_yaw):.1f}°')
            self.get_logger().info(f'   Topic: {self.odom_topic_found}')
        else:
            self.get_logger().warn('🚜 Forklift: ⚠️ SIN ODOMETRÍA')
        
        self.get_logger().info('')
        self.get_logger().info('📦 Pallets:')
        for name, (px, py) in self.pallet_positions.items():
            dist = math.sqrt((px - self.forklift_x)**2 + (py - self.forklift_y)**2)
            status = '🔗' if name == self.grasped_pallet else '  '
            self.get_logger().info(f'   {status} {name}: ({px:.1f}, {py:.1f}) - {dist:.1f}m')
        self.get_logger().info('═' * 50)

    def set_object_pose(self, name, x, y, z, yaw):
        """Mueve un objeto en mvsim"""
        if not self.connected:
            return False
        
        try:
            req = SrvSetPose_pb2.SrvSetPose()
            req.objectId = name
            req.pose.x = x
            req.pose.y = y
            req.pose.z = z
            req.pose.yaw = yaw
            req.pose.pitch = 0.0
            req.pose.roll = 0.0
            self.client.callService('set_pose', req.SerializeToString())
            return True
        except Exception as e:
            self.get_logger().error(f'Error moviendo objeto: {e}')
            return False

    def toggle_lift(self):
        """Alterna entre enganchar y soltar"""
        if self.is_lifted:
            self.release()
        else:
            self.grasp()

    def grasp(self):
        """Engancha el pallet más cercano"""
        if self.grasped_pallet:
            self.get_logger().warn(f'Ya tienes enganchado: {self.grasped_pallet}')
            return
        
        if not self.odom_received:
            self.get_logger().error('⚠️ No se recibe odometría del forklift!')
            self.get_logger().error('   Verifica que la simulación está corriendo.')
            return
        
        if not self.connected:
            if not self.connect_mvsim():
                return
        
        nearest, dist = self.find_nearest_pallet()
        
        if nearest is None:
            self.get_logger().warn('No hay pallets disponibles')
            return
        
        if dist > MAX_GRASP_DISTANCE:
            self.get_logger().warn(f'Pallet {nearest} muy lejos ({dist:.1f}m > {MAX_GRASP_DISTANCE}m)')
            self.get_logger().info(f'   Tu posición: ({self.forklift_x:.2f}, {self.forklift_y:.2f})')
            self.get_logger().info(f'   Pallet en: {self.pallet_positions[nearest]}')
            return
        
        self.grasped_pallet = nearest
        self.is_lifted = True
        
        # Publicar estado
        msg = Bool()
        msg.data = True
        self.lift_state_pub.publish(msg)
        
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info(f'🔼 ELEVADO: {nearest} (distancia: {dist:.1f}m)')
        self.get_logger().info('   El pallet ahora sigue al forklift')
        self.get_logger().info('═' * 50)

    def release(self):
        """Suelta el pallet"""
        if not self.grasped_pallet:
            self.get_logger().warn('No hay pallet enganchado')
            return
        
        released = self.grasped_pallet
        self.grasped_pallet = None
        self.is_lifted = False
        
        # Publicar estado
        msg = Bool()
        msg.data = False
        self.lift_state_pub.publish(msg)
        
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info(f'🔽 SOLTADO: {released}')
        self.get_logger().info('   El pallet queda en su posición actual')
        self.get_logger().info('═' * 50)

    def update_pallet_position(self):
        """Actualiza posición del pallet enganchado"""
        if not self.is_lifted or not self.grasped_pallet:
            return
        
        if not self.odom_received:
            return
        
        # Calcular posición delante del forklift
        cos_yaw = math.cos(self.forklift_yaw)
        sin_yaw = math.sin(self.forklift_yaw)
        
        global_x = self.forklift_x + GRASP_OFFSET_X * cos_yaw
        global_y = self.forklift_y + GRASP_OFFSET_X * sin_yaw
        
        # Actualizar posición guardada
        self.pallet_positions[self.grasped_pallet] = (global_x, global_y)
        
        # Mover en mvsim
        self.set_object_pose(
            self.grasped_pallet,
            global_x,
            global_y,
            GRASP_OFFSET_Z,
            self.forklift_yaw + math.pi / 2.0
        )


def get_key(timeout=0.1):
    """Lee una tecla del teclado"""
    try:
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    except Exception:
        time.sleep(timeout)
        return ''


def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    
    # Spinner en thread separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n" + "=" * 60)
    print("  🎮 L=Lift, P=Positions, Q=Quit")
    print("=" * 60 + "\n")
    
    try:
        while node.running:
            key = get_key()
            
            if key == '\x1b' or key.lower() == 'q':  # ESC or Q
                print("\n🛑 Saliendo...")
                break
            elif key.lower() == 'l':
                node.toggle_lift()
            elif key.lower() == 'p':
                node.show_positions()
            
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Lift controller cerrado.\n")


if __name__ == '__main__':
    main()
