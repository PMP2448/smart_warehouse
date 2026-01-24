#!/usr/bin/env python3
"""
Pallet Visual Servoing - Prototipo v2
Detecta el pallet por color y centra el robot mientras se aproxima.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np


class PalletVisualServoing(Node):
    def __init__(self):
        super().__init__('pallet_visual_servoing')
        self.bridge = CvBridge()
        
        # Parámetros
        self.linear_speed = 0.15  # m/s - velocidad de aproximación
        self.angular_gain = 0.003  # Ganancia para corrección angular
        self.target_area = 80000  # Área objetivo del pallet (cuando empezar inserción)
        self.min_pallet_area = 2000  # Área mínima para detectar el pallet
        
        # Suscriptor a la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/aruco_cam/image_raw',
            self.image_callback,
            10
        )
        
        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publicador de imagen de debug
        self.debug_pub = self.create_publisher(Image, '/pallet_detection/debug', 10)
        
        self.saved_debug = False
        
        # Estados: 'approach', 'insert', 'done'
        self.state = 'approach'
        self.insert_start_time = None
        self.insert_duration = 8.0  # segundos para insertar debajo del pallet
        
        # Debug
        self.frame_count = 0
        self.debug_folder = '/tmp/pallet_debug/'
        import os
        os.makedirs(self.debug_folder, exist_ok=True)
        self.get_logger().info(f'📁 Debug images will be saved to {self.debug_folder}')
        
        self.get_logger().info('🎯 Pallet Visual Servoing v2 started!')
        self.get_logger().info('📷 Listening to /camera_low/image_raw')
        self.get_logger().info('🚗 Publishing to /cmd_vel')

    def image_callback(self, msg):
        try:
            # Convertir a OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Flip vertical (bug de Mvsim)
            cv_image = cv2.flip(cv_image, 0)
            
            # Detectar el pallet
            pallet_info, debug_image = self.detect_pallet(cv_image)
            
            # Guardar debug cada 30 frames
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                filename = f'{self.debug_folder}frame_{self.frame_count:05d}_{self.state}.png'
                cv2.imwrite(filename, debug_image)
                self.get_logger().info(f'📸 Saved: {filename}')
            
            # Calcular comando de velocidad
            twist = self.calculate_velocity(pallet_info, cv_image.shape)
            
            # Publicar velocidad
            self.cmd_pub.publish(twist)
            
            # Publicar imagen de debug
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def detect_pallet(self, image):
        """
        Detecta el pallet por su color madera.
        Retorna info del pallet y una imagen de debug.
        """
        debug = image.copy()
        h, w = image.shape[:2]
        img_center_x = w // 2
        
        # SOLO analizar la mitad inferior de la imagen (donde está el pallet)
        roi_y_start = h // 2
        roi = image[roi_y_start:, :]
        
        # Convertir a HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Detectar el color de la madera del pallet (beige/marrón claro)
        # Ampliamos el rango para capturar más tonos
        lower_wood = np.array([10, 20, 80])
        upper_wood = np.array([40, 180, 255])
        wood_mask = cv2.inRange(hsv, lower_wood, upper_wood)
        
        # Operaciones morfológicas
        kernel = np.ones((7, 7), np.uint8)
        wood_mask = cv2.morphologyEx(wood_mask, cv2.MORPH_CLOSE, kernel)
        wood_mask = cv2.morphologyEx(wood_mask, cv2.MORPH_OPEN, kernel)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(wood_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        pallet_info = None
        
        if len(contours) > 0:
            # Encontrar el contorno más grande
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > self.min_pallet_area:
                # Obtener bounding box (en coordenadas del ROI)
                x, y, bw, bh = cv2.boundingRect(largest)
                
                # Ajustar Y al sistema de coordenadas completo
                y_full = y + roi_y_start
                
                pallet_center_x = x + bw // 2
                pallet_center_y = y_full + bh // 2
                
                # Dibujar detección
                cv2.rectangle(debug, (x, y_full), (x + bw, y_full + bh), (0, 255, 255), 3)
                cv2.circle(debug, (pallet_center_x, pallet_center_y), 10, (0, 0, 255), -1)
                
                # Línea central de la imagen
                cv2.line(debug, (img_center_x, 0), (img_center_x, h), (255, 255, 0), 2)
                
                # Línea del centro del pallet
                cv2.line(debug, (pallet_center_x, y_full), (pallet_center_x, y_full + bh), (255, 0, 255), 2)
                
                # Dibujar línea que separa ROI
                cv2.line(debug, (0, roi_y_start), (w, roi_y_start), (0, 128, 255), 2)
                
                # Calcular error
                error = pallet_center_x - img_center_x
                
                # Info en pantalla
                cv2.putText(debug, f'Error: {error}px', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(debug, f'Area: {area:.0f}', (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(debug, f'Target: {self.target_area}', (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(debug, f'State: {self.state}', (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                pallet_info = {
                    'center_x': pallet_center_x,
                    'center_y': pallet_center_y,
                    'area': area,
                    'error': error
                }
        
        if pallet_info is None:
            cv2.putText(debug, 'NO PALLET DETECTED', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return pallet_info, debug

    def calculate_velocity(self, pallet_info, img_shape):
        """
        Calcula la velocidad basándose en la detección del pallet.
        """
        twist = Twist()
        import time
        
        if self.state == 'done':
            self.get_logger().info('🏁 Done - robot stopped')
            return twist
        
        if self.state == 'insert':
            # Fase de inserción - avanzar recto durante X segundos
            elapsed = time.time() - self.insert_start_time
            if elapsed < self.insert_duration:
                twist.linear.x = self.linear_speed
                self.get_logger().info(f'🔄 Inserting... {elapsed:.1f}s / {self.insert_duration}s')
            else:
                self.state = 'done'
                self.get_logger().info('✅ INSERTION COMPLETE!')
            return twist
        
        # Estado 'approach'
        if pallet_info is not None:
            error = pallet_info['error']
            area = pallet_info['area']
            
            # Corrección angular proporcional al error
            twist.angular.z = error * self.angular_gain
            
            # Limitar velocidad angular
            twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
            
            # Si el área es pequeña, avanzar; si es grande, pasar a insertar
            if area < self.target_area:
                twist.linear.x = self.linear_speed
                self.get_logger().info(f'🎯 Approaching: error={error}px, area={area:.0f}, w={twist.angular.z:.3f}')
            else:
                # Transición a fase de inserción
                self.state = 'insert'
                self.insert_start_time = time.time()
                self.get_logger().info(f'🎯 Target reached! Starting insertion phase...')
                twist.linear.x = self.linear_speed
        else:
            # No se detecta el pallet - parar
            self.get_logger().warn('⚠️ No pallet detected - stopping')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        return twist


def main(args=None):
    rclpy.init(args=args)
    node = PalletVisualServoing()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar el robot al cerrar
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
