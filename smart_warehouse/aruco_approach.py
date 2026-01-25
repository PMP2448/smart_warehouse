#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from tf2_ros import Buffer, TransformListener, TransformException
import math
import sys
import time

# --- CONSTANTES DE ESTADO ---
ST_IDLE              = -1
ST_SEARCHING         = 0 
ST_APPROACHING       = 1 
ST_APPROACHING_FINAL = 2
ST_FINISHED          = 3

# 90º -> mirando al ArUco
# 0º -> perpendicular al ArUco (derecha)
# -90º -> mirando en sentido contrario al ArUco
yaw_offset = math.radians(-90.0) 

class ArucoApproach(Node):
    def __init__(self):
        super().__init__('aruco_approach')
        
        # Parámetros (Forklift Config)
        self.declare_parameter('target_dist', 1.40)      # Distancia final
        self.declare_parameter('pre_target_dist', 2.0)   # Distancia pre-aproximación
        self.declare_parameter('tolerance_dist', 0.01)
        self.declare_parameter('tolerance_lat', 0.04)
        self.declare_parameter('tolerance_yaw', math.radians(3.0))
        self.declare_parameter('max_v', 0.2)
        self.declare_parameter('max_w', 1.0)
        self.declare_parameter('max_v2', 0.1)
        self.declare_parameter('max_w2', 1.0)
        self.declare_parameter('k_v', 0.18)
        self.declare_parameter('k_w_lat', 0.4)
        self.declare_parameter('k_w_angle', 0.4)
        
        # Cargar valores
        self.target_dist = self.get_parameter('target_dist').value
        self.pre_target_dist = self.get_parameter('pre_target_dist').value
        self.tolerance_dist = self.get_parameter('tolerance_dist').value
        self.tolerance_lat = self.get_parameter('tolerance_lat').value
        self.tolerance_yaw = self.get_parameter('tolerance_yaw').value
        self.k_v = self.get_parameter('k_v').value
        self.k_w_lat = self.get_parameter('k_w_lat').value
        self.k_w_angle = self.get_parameter('k_w_angle').value
        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Pubs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'docking_status', 10)
        
        # Subs
        self.trigger_sub = self.create_subscription(String, 'docking_trigger', self.trigger_callback, 10)
        
        # Service Client for Aruco ID
        self.param_client = self.create_client(SetParameters, '/aruco_single/set_parameters')
        
        self.state = ST_IDLE
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        
        self.get_logger().info("Aruco Approach Node Ready (State: IDLE)")

    def trigger_callback(self, msg):
        cmd = msg.data # Expected "START:P1" or "STOP"
        self.get_logger().info(f"Received docking trigger: {cmd}")
        
        if cmd == "STOP":
            self.state = ST_IDLE
            self.stop_robot()
            self.publish_status("STOPPED")
            return
            
        if cmd.startswith("START:"):
            loc = cmd.split(":")[1]
            if loc.startswith("P"):
                try:
                    p_num = int(loc[1:]) # P1 -> 1
                    aruco_id = p_num - 1 # ID 0
                    self.set_aruco_id(aruco_id)
                    self.state = ST_SEARCHING
                    self.get_logger().info(f"Starting Approach for {loc} (ArUco ID {aruco_id})")
                    self.publish_status("SEARCHING")
                except ValueError:
                    self.get_logger().error(f"Invalid pallet format: {loc}")
            else:
                 self.get_logger().error(f"Unknown location type: {loc}")

    def set_aruco_id(self, id_val):
        """Dynamic reconfiguration of Aruco Node"""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
             self.get_logger().warn("Aruco param service not available - cannot switch ID")
             return

        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=id_val)
        param = Parameter(name='marker_id', value=val)
        req.parameters.append(param)
        
        future = self.param_client.call_async(req)
        # We don't wait for result to avoid blocking main thread, assuming it works

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def quaternion_to_yaw(self, q):
        yaw = math.atan2(2.0 * (q.w * q.z - q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

    def get_aruco_tf(self):
        # Usar frame filtrado
        from_frame = 'aruco_detectado_filtered'
        to_frame   = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                from_frame, to_frame, rclpy.time.Time())

            error_x = t.transform.translation.x
            error_z = t.transform.translation.z
            yaw_rad = self.quaternion_to_yaw(t.transform.rotation) + yaw_offset
            
            return error_x, error_z, yaw_rad

        except TransformException:
            return None

    def control_loop(self):
        if self.state == ST_IDLE or self.state == ST_FINISHED:
            return

        data = self.get_aruco_tf()
        cmd = Twist()

        if data is None:
            # Si estamos buscando, seguimos buscando
            # Si estabamos aproximando y se pierde... parar por seguridad
            if self.state != ST_SEARCHING:
                self.stop_robot()
            return

        rx, rz, ryaw = data
        
        # --- MAQUINA ESTADOS ---
        if self.state == ST_SEARCHING:
            self.get_logger().info("ArUco Detected. Switching to APPROACHING")
            self.state = ST_APPROACHING
            self.publish_status("APPROACHING")

        elif self.state == ST_APPROACHING:
            # Fase hasta pre_target
            dist_error = rz - self.pre_target_dist
            lat_error = rx
            angle_error = ryaw
            
            # Control Logic
            v = 0.0
            
            # Solo avanzar si estamos "decente" alineados
            if abs(lat_error) < 0.2: 
                v = self.max_v
                if dist_error < 0: # Ya nos pasamos
                    v = 0.0
            
            w = self.k_w_lat * lat_error - self.k_w_angle * angle_error
            
            if abs(dist_error) < self.tolerance_dist:
                self.state = ST_APPROACHING_FINAL
                self.publish_status("FINAL_APPROACH")
                self.get_logger().info("Pre-target reached. Starting Final Approach.")
            
            cmd.linear.x = v
            cmd.angular.z = w

        elif self.state == ST_APPROACHING_FINAL:
            # Fase fina hasta target
            dist_error = rz - self.target_dist
            lat_error = rx
            angle_error = ryaw
            
            v = self.k_v * dist_error
            w = self.k_w_lat * lat_error - self.k_w_angle * angle_error
            
            if abs(dist_error) < self.tolerance_dist:
                self.state = ST_FINISHED
                self.stop_robot()
                self.publish_status("FINISHED")
                self.get_logger().info("Target Reached. Docking Complete.")
                return

            cmd.linear.x = v
            cmd.angular.z = w

        # Saturacion
        cmd.linear.x = float(max(min(cmd.linear.x, self.max_v), -self.max_v))
        cmd.angular.z = float(max(min(cmd.angular.z, self.max_w), -self.max_w))
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoApproach()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
