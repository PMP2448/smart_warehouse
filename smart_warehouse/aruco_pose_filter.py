#!/usr/bin/env python3
"""
Filtro Suavizante para Pose del ArUco

Lee el topic '/aruco_single/pose' y publica el frame TF 'aruco_detectado_filtered'
aplicando un filtro de media móvil para suavizar el ruido.

Uso:
    ros2 run smart_warehouse aruco_pose_filter

En tu código de docking, solo cambia:
    from_frame = 'aruco_detectado'  ->  from_frame = 'aruco_detectado_filtered'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
from collections import deque
import math
import sys


def quaternion_to_euler(q):
    """Convierte quaternion a ángulos de Euler (roll, pitch, yaw)"""
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convierte ángulos de Euler a quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def normalize_angle(angle):
    """Normaliza un ángulo a [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class ArucoSmoothFilter(Node):
    def __init__(self):
        super().__init__('aruco_pose_filter')
        
        # 1. PARÁMETROS DE FILTRADO
        self.declare_parameter('window_size', 20)  # Tamaño de la ventana de media móvil
        self.declare_parameter('input_topic', '/aruco_single/pose')
        self.declare_parameter('output_frame', 'aruco_detectado_filtered')
        self.declare_parameter('world_frame', 'map')  # Frame fijo del mundo
        
        # Cargar parámetros
        self.window_size = self.get_parameter('window_size').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        
        # 2. SUSCRIPCIÓN AL TOPIC DE POSE
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            10
        )
        
        # 3. CONFIGURACIÓN TF (LECTURA Y ESCRITURA)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 4. HISTORIAL DE DATOS (ventana de media móvil)
        self.pos_history_x = deque(maxlen=self.window_size)
        self.pos_history_y = deque(maxlen=self.window_size)
        self.pos_history_z = deque(maxlen=self.window_size)
        self.rot_history_roll = deque(maxlen=self.window_size)
        self.rot_history_pitch = deque(maxlen=self.window_size)
        self.rot_history_yaw = deque(maxlen=self.window_size)
        
        # Estadísticas
        self.total_frames = 0
        
        self.get_logger().info(f"Filtro Suavizante de ArUco iniciado:")
        self.get_logger().info(f"  - Input topic: '{self.input_topic}'")
        self.get_logger().info(f"  - Output frame: '{self.output_frame}'")
        self.get_logger().info(f"  - World frame: '{self.world_frame}'")
        self.get_logger().info(f"  - Window size: {self.window_size}")
    
    def circular_mean(self, angles):
        """Calcula la media de ángulos (maneja wraparound correctamente)"""
        sin_sum = sum(math.sin(a) for a in angles)
        cos_sum = sum(math.cos(a) for a in angles)
        return math.atan2(sin_sum, cos_sum)
    
    def pose_callback(self, msg):
        """Callback para cada mensaje de pose"""
        self.total_frames += 1
        
        # Transformar la pose al frame del mundo para que quede fija
        try:
            # Transformar pose del frame original al frame del mundo
            pose_in_world = self.tf_buffer.transform(
                msg, 
                self.world_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            if self.total_frames % 100 == 1:  # Log cada 100 frames
                self.get_logger().warn(f"No se pudo transformar a '{self.world_frame}': {e}")
            return
        
        # Extraer posición y rotación en el frame del mundo
        new_pos = (
            pose_in_world.pose.position.x,
            pose_in_world.pose.position.y,
            pose_in_world.pose.position.z
        )
        new_rot = quaternion_to_euler(pose_in_world.pose.orientation)
        
        # Añadir al historial
        self.pos_history_x.append(new_pos[0])
        self.pos_history_y.append(new_pos[1])
        self.pos_history_z.append(new_pos[2])
        self.rot_history_roll.append(new_rot[0])
        self.rot_history_pitch.append(new_rot[1])
        self.rot_history_yaw.append(new_rot[2])
        
        # Calcular media móvil
        filtered_pos = (
            sum(self.pos_history_x) / len(self.pos_history_x),
            sum(self.pos_history_y) / len(self.pos_history_y),
            sum(self.pos_history_z) / len(self.pos_history_z),
        )
        
        # Para rotación usar media circular
        filtered_rot = (
            self.circular_mean(self.rot_history_roll),
            self.circular_mean(self.rot_history_pitch),
            self.circular_mean(self.rot_history_yaw),
        )
        
        # Publicar el transform filtrado (desde world_frame)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.world_frame  # Frame fijo del mundo
        tf_msg.child_frame_id = self.output_frame
        
        tf_msg.transform.translation.x = filtered_pos[0]
        tf_msg.transform.translation.y = filtered_pos[1]
        tf_msg.transform.translation.z = filtered_pos[2]
        tf_msg.transform.rotation = euler_to_quaternion(
            filtered_rot[0],
            filtered_rot[1],
            filtered_rot[2]
        )
        
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # Log sucinto
        sys.stdout.write("\033[K")
        sys.stdout.write(
            f"\r✅ Frame {self.total_frames} (win={len(self.pos_history_x)}) | "
            f"Pos: ({filtered_pos[0]:+.3f}, {filtered_pos[1]:+.3f}, {filtered_pos[2]:+.3f}m) | "
            f"Rot: ({math.degrees(filtered_rot[0]):+.1f}°, {math.degrees(filtered_rot[1]):+.1f}°, {math.degrees(filtered_rot[2]):+.1f}°)"
        )
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    filter_node = ArucoSmoothFilter()
    
    try:
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        filter_node.get_logger().info(f"\nTotal frames procesados: {filter_node.total_frames}")
    finally:
        filter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
