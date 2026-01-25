#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import datetime

class InterfaceNode(Node):
    def __init__(self, gui_callback):
        super().__init__('interface_node')
        self.gui_callback = gui_callback
        
        # Publishers
        self.pub_tarea = self.create_publisher(String, 'tarea', 10)
        self.pub_nav = self.create_publisher(String, 'navegacion', 10)
        self.pub_orient = self.create_publisher(String, 'orientacion', 10)
        self.pub_grasp = self.create_publisher(String, 'agarre', 10)
        self.pub_deposit = self.create_publisher(String, 'deposicion', 10)
        self.pub_status = self.create_publisher(String, 'estado', 10)
        self.pub_goal = self.create_publisher(String, 'navigation_goal', 10)
        self.pub_docking = self.create_publisher(String, 'docking_trigger', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.sub_nav_status = self.create_subscription(String, 'navigation_status', self.nav_status_callback, 10)
        self.sub_dock_status = self.create_subscription(String, 'docking_status', self.dock_status_callback, 10)
        self.sub_prev_node = self.create_subscription(String, 'previous_node', self.prev_node_callback, 10)
        
        # Store previous node from navigation
        self.last_previous_node = None
        
        self.get_logger().info('Interface Node Started')

    def nav_status_callback(self, msg):
        self.get_logger().info(f'RECV [navigation_status]: {msg.data}')
        if self.gui_callback:
            self.gui_callback("NAV", msg.data)

    def dock_status_callback(self, msg):
        self.get_logger().info(f'RECV [docking_status]: {msg.data}')
        if self.gui_callback:
            self.gui_callback("DOCK", msg.data)

    def prev_node_callback(self, msg):
        """Store the previous node from navigation path"""
        self.last_previous_node = msg.data
        self.get_logger().info(f'RECV [previous_node]: {msg.data}')

    def publish_message(self, topic, payload):
        msg = String()
        msg.data = payload
        
        if topic == 'tarea':
            self.pub_tarea.publish(msg)
        elif topic == 'navegacion':
            self.pub_nav.publish(msg)
        elif topic == 'orientacion':
            self.pub_orient.publish(msg)
        elif topic == 'agarre':
            self.pub_grasp.publish(msg)
        elif topic == 'deposicion':
            self.pub_deposit.publish(msg)
        elif topic == 'estado':
            self.pub_status.publish(msg)
        elif topic == 'navigation_goal':
            self.pub_goal.publish(msg)
        elif topic == 'docking_trigger':
            self.pub_docking.publish(msg)
            
        self.get_logger().info(f'PUB [{topic}]: {payload}')

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)

class ControlPanel:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.ros_node.gui_callback = self.on_status_received # Link callback
        
        self.root.title("HJP LOGÍSTICA - Control de Flota")
        self.root.geometry("535x750")
        
        # --- ESTILOS ---
        self.COLOR_BG = "#1e293b"
        self.COLOR_TEXT = "#ffffff"
        self.COLOR_ACCENT = "#38bdf8"
        self.COLOR_STATUS = "#fbbf24"
        
        self.root.configure(bg=self.COLOR_BG) 

        self.is_active = False
        self.mission_phase = 0 
        # Phases:
        # 1: Nav -> Origin
        # 2: Approach Origin (Blind)
        # 3: Pick
        # 4: Reverse to previous node (navigated)
        # 5: Nav -> Target
        # 6: Approach Target (Blind)
        # 7: Drop
        # 8: Reverse to previous node (navigated)
        # 9: Nav -> Home
        
        self.current_origin = ""
        self.current_target = ""
        self.origin_previous_node = ""   # Nodo anterior al origen
        self.target_previous_node = ""   # Nodo anterior al destino

        # --- FUENTES ---
        font_title = ("Helvetica", 22, "bold")
        font_label = ("Arial", 12, "bold") 
        font_input = ("Arial", 14)
        font_btn_start = ("Arial", 14, "bold")
        font_status_title = ("Arial", 12)
        font_status_val = ("Courier", 26, "bold")   
        font_btn_stop = ("Arial", 14, "bold")
        font_console = ("Consolas", 10) 

        self.root.option_add('*TCombobox*Listbox.font', font_input)

        # --- UBICACIONES ---
        # --- UBICACIONES (Dynamic from Graph) ---
        self.posibles_ubicaciones = []
        try:
            import json
            import os
            from ament_index_python.packages import get_package_share_directory
            
            pkg_dir = get_package_share_directory('smart_warehouse')
            graph_file = os.path.join(pkg_dir, 'graphs', 'warehouse_graph.geojson')
            
            with open(graph_file, 'r') as f:
                data = json.load(f)
                for feature in data.get('features', []):
                    if feature['geometry']['type'] == 'Point':
                        name = feature['properties'].get('name')
                        if name:
                            self.posibles_ubicaciones.append(name)
            
            self.posibles_ubicaciones.sort()
            # Ensure HOME is first if present
            if "HOME" in self.posibles_ubicaciones:
                self.posibles_ubicaciones.remove("HOME")
                self.posibles_ubicaciones.insert(0, "HOME")
                
        except Exception as e:
            print(f"Error loading graph: {e}")
            self.posibles_ubicaciones = ["HOME", "ESTANTERIA_1", "ESTANTERIA_2"] # Fallback

        # --- GUI ELEMENTS ---
        lbl_title = tk.Label(root, text="CONTROL DE MISIÓN", font=font_title, bg=self.COLOR_BG, fg=self.COLOR_ACCENT)
        lbl_title.pack(pady=(30, 20)) 

        frame_inputs = tk.Frame(root, bg=self.COLOR_BG)
        frame_inputs.pack(pady=10) 

        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TCombobox", fieldbackground="#f1f5f9", background="#94a3b8", foreground="#0f172a", arrowsize=20)

        # Origen
        tk.Label(frame_inputs, text="Origen (ID_LOC1):", font=font_label, bg=self.COLOR_BG, fg=self.COLOR_TEXT).grid(row=0, column=0, padx=10, pady=10, sticky="e")
        self.combo_origin = ttk.Combobox(frame_inputs, values=self.posibles_ubicaciones, font=font_input, width=20, state="readonly")
        self.combo_origin.grid(row=0, column=1, padx=10, pady=10)
        self.combo_origin.current(0) 

        # Destino
        tk.Label(frame_inputs, text="Destino (ID_LOC2):", font=font_label, bg=self.COLOR_BG, fg=self.COLOR_TEXT).grid(row=1, column=0, padx=10, pady=10, sticky="e")
        self.combo_target = ttk.Combobox(frame_inputs, values=self.posibles_ubicaciones, font=font_input, width=20, state="readonly")
        self.combo_target.grid(row=1, column=1, padx=10, pady=10)
        self.combo_target.current(1) 

        # Boton Start
        self.btn_start = tk.Button(root, text="INICIAR TAREA", command=self.start_mission, 
                                   bg=self.COLOR_ACCENT, fg="black", font=font_btn_start, 
                                   height=2, width=18, cursor="hand2")
        self.btn_start.pack(pady=20)

        # --- NAVEGACIÓN DIRECTA ---
        frame_direct = tk.LabelFrame(root, text="Navegación Directa", font=font_label, bg=self.COLOR_BG, fg=self.COLOR_ACCENT, bd=2)
        frame_direct.pack(pady=5, padx=20, fill="x")

        self.combo_direct = ttk.Combobox(frame_direct, values=self.posibles_ubicaciones, font=font_input, width=15, state="readonly")
        self.combo_direct.pack(side="left", padx=10, pady=10)
        self.combo_direct.current(0)

        self.btn_go = tk.Button(frame_direct, text="IR AHORA", command=self.go_direct,
                                bg="#22c55e", fg="black", font=("Arial", 12, "bold"),
                                height=1, width=10, cursor="hand2")
        self.btn_go.pack(side="right", padx=10, pady=10)

        # Status
        self.lbl_status_title = tk.Label(root, text="ESTADO DEL ROBOT:", font=font_status_title, bg=self.COLOR_BG, fg="#94a3b8")
        self.lbl_status_title.pack(pady=(10, 5))
        
        self.lbl_current_state = tk.Label(root, text="REPOSO", font=font_status_val, bg=self.COLOR_BG, fg=self.COLOR_STATUS)
        self.lbl_current_state.pack(pady=10)

        # Log
        lbl_log = tk.Label(root, text="REGISTRO DE COMUNICACIONES (TOPICS):", font=("Arial", 10, "bold"), bg=self.COLOR_BG, fg="#cbd5e1")
        lbl_log.pack(pady=(20, 5), anchor="w", padx=20)

        self.console_log = scrolledtext.ScrolledText(root, height=8, bg="#0f172a", fg="#22c55e", font=font_console, state='disabled', borderwidth=1, relief="solid")
        self.console_log.pack(padx=20, pady=5, fill="x")

        # Stop
        btn_stop = tk.Button(root, text="PARADA DE EMERGENCIA", command=self.emergency_stop, 
                             bg="#ef4444", fg="white", font=font_btn_stop, 
                             height=2, width=25, cursor="hand2")
        btn_stop.pack(side=tk.BOTTOM, pady=30)

    def log_message(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        full_msg = f"[{timestamp}] {message}\n"
        self.console_log.config(state='normal')
        self.console_log.insert(tk.END, full_msg)
        self.console_log.see(tk.END)
        self.console_log.config(state='disabled')

    def publicar_mensaje(self, topic, payload):
        self.ros_node.publish_message(topic, payload)
        msg = f"PUB [{topic}]: {payload}"
        self.log_message(msg)

    def go_direct(self):
        target = self.combo_direct.get()
        self.log_message(f"--> ORDEN DIRECTA: Ir a {target}")
        
        # Stop any active mission
        self.is_active = False 
        self.mission_phase = 0
        
        self.lbl_current_state.config(text=f"YENDO A {target}", fg=self.COLOR_STATUS)
        
        # Ensure Nav is ON
        self.publicar_mensaje("navegacion", "STATUS: ON")
        # Send Goal
        self.publicar_mensaje("navigation_goal", target)

    def start_mission(self):
        origin = self.combo_origin.get()
        target = self.combo_target.get()
        
        if origin == target:
            self.log_message("⚠️ WARN: Origen y Destino idénticos")
        
        self.log_message(f"--> ORDEN ENVIADA: {origin} -> {target}")
        
        self.current_origin = origin
        self.current_target = target
        
        # Limpiar nodos anteriores de misiones previas
        self.origin_previous_node = ""
        self.target_previous_node = ""
        self.ros_node.last_previous_node = None
        
        self.is_active = True
        self.mission_phase = 1 # Start Phase 1: Go to Origin
        
        self.publicar_mensaje("tarea", "STATUS: ON")
        self.execute_phase()

    def perform_blind_move(self, speed, duration_ms, next_phase_delay_ms=1000):
        """Executes a blind movement (open loop) for a duration"""
        if not self.is_active: return
        
        import time as time_module
        end_time = time_module.time() + duration_ms / 1000.0
        
        def keep_publishing():
            if time_module.time() < end_time and self.is_active:
                self.ros_node.publish_cmd_vel(speed, 0.0)
                self.root.after(100, keep_publishing)  # Publish every 100ms
            else:
                self.ros_node.publish_cmd_vel(0.0, 0.0)  # Stop
                self.root.after(next_phase_delay_ms, self.advance_phase)
        
        keep_publishing()

    def execute_phase(self):
        if not self.is_active: return

        if self.mission_phase == 1:
            # Phase 1: Navigate to Origin
            self.lbl_current_state.config(text="NAV -> ORIGEN", fg=self.COLOR_STATUS)
            self.publicar_mensaje("estado", "FASE 1: Yendo a Origen")
            self.publicar_mensaje("navegacion", "STATUS: ON")
            self.publicar_mensaje("navigation_goal", self.current_origin)
            
        elif self.mission_phase == 2:
            # Phase 2: Approach Origin (ArUco Docking)
            self.lbl_current_state.config(text=f"DOCKING -> {self.current_origin}", fg="#a855f7")
            self.publicar_mensaje("estado", "FASE 2: Docking ArUco")
            self.publicar_mensaje("navegacion", "STATUS: OFF")
            # Trigger Docking
            self.publicar_mensaje("docking_trigger", f"START:{self.current_origin}")
            
        elif self.mission_phase == 3:
            # Phase 3: Pick
            self.lbl_current_state.config(text="RECOGIENDO CARGA", fg="#eab308")
            self.publicar_mensaje("estado", "FASE 3: Operación de Carga")
            self.publicar_mensaje("agarre", "STATUS: ON")
            # Wait 2s for picking
            self.root.after(2000, self.advance_phase)
            
        elif self.mission_phase == 4:
            # Phase 4: Retroceder al nodo anterior (navegación reversa)
            self.lbl_current_state.config(text="RETROCEDIENDO...", fg="#a855f7")
            self.publicar_mensaje("estado", "FASE 4: Retroceso al nodo anterior")
            prev_node = self.origin_previous_node
            self.log_message(f"📍 Nodo anterior origen: '{prev_node}'")
            
            if prev_node and prev_node.strip():
                self.publicar_mensaje("navegacion", "STATUS: ON")
                self.publicar_mensaje("navigation_goal", f"REVERSE:{prev_node}")
                self.log_message(f"🔄 Navegación reversa a: {prev_node}")
            else:
                # Fallback: blind move si no hay nodo anterior
                self.log_message("⚠️ Sin nodo anterior, usando movimiento ciego")
                self.perform_blind_move(-0.5, 2000)
            
        elif self.mission_phase == 5:
            # Phase 5: Navigate to Target
            self.lbl_current_state.config(text=f"NAV -> {self.current_target}", fg=self.COLOR_STATUS)
            self.publicar_mensaje("estado", f"FASE 5: Yendo a {self.current_target}")
            self.publicar_mensaje("agarre", "STATUS: OFF")
            self.publicar_mensaje("navegacion", "STATUS: ON")
            self.publicar_mensaje("navigation_goal", self.current_target)
            
        elif self.mission_phase == 6:
            # Phase 6: Approach Target (ArUco Docking)
            self.lbl_current_state.config(text=f"DOCKING -> {self.current_target}", fg="#a855f7")
            self.publicar_mensaje("estado", "FASE 6: Docking ArUco")
            self.publicar_mensaje("navegacion", "STATUS: OFF")
            # Trigger Docking
            self.publicar_mensaje("docking_trigger", f"START:{self.current_target}")
            
        elif self.mission_phase == 7:
            # Phase 7: Drop
            self.lbl_current_state.config(text="DEPOSITANDO CARGA", fg="#eab308")
            self.publicar_mensaje("estado", "FASE 7: Operación de Descarga")
            self.publicar_mensaje("deposicion", "STATUS: ON")
            # Wait 3s for dropping
            self.root.after(1700, self.advance_phase)
            
        elif self.mission_phase == 8:
            # Phase 8: Retroceder al nodo anterior (navegación reversa)
            self.lbl_current_state.config(text="RETROCEDIENDO...", fg="#a855f7")
            self.publicar_mensaje("estado", "FASE 8: Retroceso al nodo anterior")
            prev_node = self.target_previous_node
            self.log_message(f"📍 Nodo anterior target: '{prev_node}'")
            
            if prev_node and prev_node.strip():
                self.publicar_mensaje("navegacion", "STATUS: ON")
                self.publicar_mensaje("navigation_goal", f"REVERSE:{prev_node}")
                self.log_message(f"🔄 Navegación reversa a: {prev_node}")
            else:
                # Fallback: blind move si no hay nodo anterior
                self.log_message("⚠️ Sin nodo anterior, usando movimiento ciego")
                self.perform_blind_move(-0.5, 2000)
            
        elif self.mission_phase == 9:
            # Phase 9: Navigate to Home
            self.lbl_current_state.config(text="NAV -> HOME", fg=self.COLOR_STATUS)
            self.publicar_mensaje("estado", "FASE 9: Volviendo a Casa")
            self.publicar_mensaje("deposicion", "STATUS: OFF")
            self.publicar_mensaje("navegacion", "STATUS: ON")
            self.publicar_mensaje("navigation_goal", "HOME")
            
        elif self.mission_phase == 10:
            # Mission Complete
            self.lbl_current_state.config(text="REPOSO", fg="#22c55e")
            self.publicar_mensaje("estado", "MISION COMPLETADA")
            self.publicar_mensaje("tarea", "STATUS: OFF")
            self.publicar_mensaje("navegacion", "STATUS: OFF")
            self.is_active = False
            self.mission_phase = 0

    def on_status_received(self, source, status):
        """Callback when Navigation or Docking Node reports status"""
        if not self.is_active: return
        
        self.log_message(f"RECV {source}: {status}")
        
        if source == "NAV" and status == "REACHED":
            # Navigation leg complete
            if self.mission_phase == 1:
                # Guardamos el nodo anterior al llegar al origen
                self.origin_previous_node = self.ros_node.last_previous_node
                self.log_message(f"📍 Nodo anterior origen: {self.origin_previous_node}")
                self.log_message("✅ Llegado a Origen. Iniciando Docking...")
                self.root.after(1000, self.advance_phase)
            elif self.mission_phase == 5:
                # Guardamos el nodo anterior al llegar al target
                self.target_previous_node = self.ros_node.last_previous_node
                self.log_message(f"📍 Nodo anterior target: {self.target_previous_node}")
                self.log_message("✅ Llegado a Target. Iniciando Docking...")
                self.root.after(1000, self.advance_phase)
            elif self.mission_phase == 4:
                # Retroceso completado desde origen
                self.log_message("✅ Retroceso completado. Avanzando...")
                self.root.after(500, self.advance_phase)
            elif self.mission_phase == 8:
                # Retroceso completado desde target
                self.log_message("✅ Retroceso completado. Avanzando...")
                self.root.after(500, self.advance_phase)
            elif self.mission_phase == 9:
                self.log_message("✅ Llegado a HOME. Misión completada.")
                self.root.after(1000, self.advance_phase)
                
        elif source == "DOCK" and status == "SUCCESS":
            if self.mission_phase == 2 or self.mission_phase == 6:
                self.log_message("✅ Docking ArUco Completado Exitosamente.")
                self.root.after(1000, self.advance_phase)

    def advance_phase(self):
        if not self.is_active: return
        self.mission_phase += 1
        self.execute_phase()

    def emergency_stop(self):
        self.publicar_mensaje("tarea", "STATUS: OFF")
        self.publicar_mensaje("navegacion", "STATUS: OFF")
        self.publicar_mensaje("docking_trigger", "STOP")
        self.is_active = False
        self.mission_phase = 0
        self.lbl_current_state.config(text="STOPPED", fg="#ef4444")

def main():
    rclpy.init()
    # Pass None initially, set in ControlPanel init
    ros_node = InterfaceNode(None)
    
    # Thread para ROS spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = ControlPanel(root, ros_node)
    root.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
