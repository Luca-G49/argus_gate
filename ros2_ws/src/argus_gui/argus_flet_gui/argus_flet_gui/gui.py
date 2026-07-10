import asyncio
import sys
import threading
from typing import Optional

import flet as ft
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from argus_interfaces.msg import ManagerStatus, PlcCommand, TeleopCommand


# ==========================================
# 1. ROS 2 NODE (Logica di Background)
# ==========================================
class ManagerGuiNode(Node):
    def __init__(self):
        super().__init__('argus_flet_gui_node')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.declare_parameter('publish_rate_hz', 50.0)
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.status_sub = self.create_subscription(ManagerStatus, 'manager_status', self.status_cb, qos)
        self.plc_cmd_pub = self.create_publisher(PlcCommand, 'plc_command', 10)
        self.teleop_pub = self.create_publisher(TeleopCommand, 'teleop_cmd', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_loop)

        self.latest_status: Optional[ManagerStatus] = None
        self.pitch_speed = 0.0
        self.yaw_speed = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.requested_mode = 0
        self.execute_action = False
        self.reset_error = False
        self.send_target = False

    def status_cb(self, msg: ManagerStatus):
        self.latest_status = msg

    def publish_mode(self, mode: int, execute: bool = False):
        cmd = PlcCommand()
        cmd.mode = mode
        cmd.exec = execute
        self.plc_cmd_pub.publish(cmd)
        self.requested_mode = mode
        self.execute_action = execute

    def publish_teleop(self,
                       pitch_speed: float = 0.0,
                       yaw_speed: float = 0.0,
                       requested_mode: int = 2,
                       target_pitch: float = 0.0,
                       target_yaw: float = 0.0,
                       send_target: bool = False,
                       execute_action: bool = False,
                       reset_error: bool = False):
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed
        self.requested_mode = requested_mode
        self.target_pitch = target_pitch
        self.target_yaw = target_yaw
        self.send_target = send_target
        self.execute_action = execute_action
        self.reset_error = reset_error

    def publish_loop(self):
        msg = TeleopCommand()
        msg.pitch_speed = self.pitch_speed
        msg.yaw_speed = self.yaw_speed
        msg.requested_mode = self.requested_mode
        msg.target_pitch = self.target_pitch
        msg.target_yaw = self.target_yaw
        msg.send_target = self.send_target
        msg.execute_action = self.execute_action
        msg.reset_error = self.reset_error
        self.teleop_pub.publish(msg)

        self.execute_action = False
        self.reset_error = False
        self.send_target = False
# ==========================================
# 2. FLET GRAPHICAL INTERFACE (Stile Tesla)
# ==========================================
class FletGui:
    def __init__(self, node: ManagerGuiNode):
        self.node = node
        self.page: Optional[ft.Page] = None
        self.jog_speed = 0.8
        self.active_jog_pitch = 0.0
        self.active_jog_yaw = 0.0

        # Componenti Monitoraggio (Colonna 1)
        self.state_text = ft.Text("ATTESA TELEMETRIA...", color="white", size=14, weight=ft.FontWeight.BOLD)
        self.teleop_conn_status = ft.Text("Teleop: --", color="#6B7280", size=11)
        self.plc_conn_status = ft.Text("PLC: --", color="#6B7280", size=11)
        
        self.pitch_pos_text = ft.Text("PITCH: --", color="#9CA3AF", size=12, weight=ft.FontWeight.W_500)
        self.yaw_pos_text = ft.Text("YAW: --", color="#9CA3AF", size=12, weight=ft.FontWeight.W_500)
        self.pitch_tgt_text = ft.Text("TARGET: --", color="#6B7280", size=11)
        self.yaw_tgt_text = ft.Text("TARGET: --", color="#6B7280", size=11)
        
        self.badge_text = ft.Text("OFFLINE", weight="bold", color="white", size=11)
        self.badge_container = ft.Container(
            content=self.badge_text,
            padding=ft.Padding(left=12, top=6, right=12, bottom=6),
            border_radius=8,
            bgcolor="#374151",
        )

        # Mappa Errore Cartesiana
        self.error_target_dot = ft.Container(
            expand=True,
            alignment=ft.Alignment(0, 0),
            content=ft.Container(width=8, height=8, border_radius=999, bgcolor="#10B981"),
        )
        self.error_actual_dot = ft.Container(
            expand=True,
            alignment=ft.Alignment(0, 0),
            content=ft.Container(width=10, height=10, border_radius=999, bgcolor="#EF4444"),
        )
        self.error_box = ft.Container(
            width=120,
            height=120,
            border_radius=12,
            bgcolor="#111111",
            border=ft.Border.all(1, "#374151"),
            content=ft.Stack([self.error_target_dot, self.error_actual_dot]),
        )
        self.error_meta_text = ft.Text("ΔP: 0.00  ΔY: 0.00", size=11, color="#6B7280", font_family="monospace")

        # Bottoni Modalità Operative (Colonna 2)
        self.btn_idle = self.create_mode_button(ft.Icons.PAUSE_ROUNDED, "IDLE", 0)
        self.btn_synch = self.create_mode_button(ft.Icons.SYNC_ALT_ROUNDED, "SYNCH", 1)
        self.btn_jog = self.create_mode_button(ft.Icons.PAN_TOOL_ALT_ROUNDED, "JOG", 2)
        self.btn_follow = self.create_mode_button(ft.Icons.TRACK_CHANGES_ROUNDED, "FOLLOW", 3)

        # Campi Target Follow
        self.pitch_target_input = ft.TextField(
            label="Pitch Target", label_style=ft.TextStyle(color="#6B7280", size=11),
            value="0.0", width=85, height=40, text_size=13, color="white",
            bgcolor="#111111", border_color="#374151", focused_border_color="#007AFF",
            content_padding=8, border_radius=8
        )
        self.yaw_target_input = ft.TextField(
            label="Yaw Target", label_style=ft.TextStyle(color="#6B7280", size=11),
            value="0.0", width=85, height=40, text_size=13, color="white",
            bgcolor="#111111", border_color="#374151", focused_border_color="#007AFF",
            content_padding=8, border_radius=8
        )

        # Slider di Velocità Jog (Colonna 3)
        self.jog_speed_label = ft.Text(f"{self.jog_speed:.2f}", weight="bold", color="white", size=12)
        self.pitch_slider = ft.Slider(
            min=0.0, max=1.5, divisions=30, value=self.jog_speed,
            active_color="#007AFF", inactive_color="#374151",
            on_change=self.on_jog_speed_change
        )
        
        # Flag di stato di sistema inferiori
        self.flags_text = ft.Text("Flags: --", size=10, color="#6B7280", overflow=ft.TextOverflow.ELLIPSIS)

    async def main(self, page: ft.Page):
        self.page = page
        page.title = "Argus Gate Mobile Dashboard"
        page.theme_mode = ft.ThemeMode.DARK
        page.bgcolor = "#111111"
        page.padding = 10
        
        # Orientamento simulato per iPhone 12 Pro Max Landscape
        page.window.width = 844
        page.window.height = 390
        page.window.resizable = False

        # --- PANNELLO 1: TELEMETRIA & STATO (SINISTRA) ---
        col_telemetry = ft.Container(
            expand=4,
            bgcolor="#1C1C1E",
            border_radius=16,
            padding=12,
            content=ft.Column([
                ft.Row([
                    ft.Column([
                        ft.Text("ARGUS GATE", size=16, weight=ft.FontWeight.BOLD, color="white"),
                        self.state_text,
                    ], spacing=2),
                    self.badge_container
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                
                ft.Divider(color="#2C2C2E", height=10),
                
                ft.Row([
                    ft.Column([
                        ft.Text("POSIZIONE ATTUALE", size=10, color="#6B7280", weight=ft.FontWeight.BOLD),
                        self.pitch_pos_text,
                        self.yaw_pos_text,
                        ft.Container(height=4),
                        ft.Text("CONNESSIONI", size=10, color="#6B7280", weight=ft.FontWeight.BOLD),
                        self.teleop_conn_status,
                        self.plc_conn_status,
                    ], spacing=2, expand=True),
                    
                    ft.Column([
                        self.error_box,
                        ft.Container(content=self.error_meta_text, alignment=ft.Alignment(0, 0))
                    ], spacing=4, horizontal_alignment=ft.CrossAxisAlignment.CENTER)
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN, vertical_alignment=ft.CrossAxisAlignment.CENTER),
            ], spacing=4)
        )

        # --- PANNELLO 2: MODALITÀ & TARGET (CENTRO) ---
        col_modes = ft.Container(
            expand=3,
            bgcolor="#1C1C1E",
            border_radius=16,
            padding=12,
            content=ft.Column([
                ft.Text("MODALITÀ OPERATIVE", size=11, color="#6B7280", weight=ft.FontWeight.BOLD),
                ft.Row([self.btn_idle, self.btn_synch], spacing=8),
                ft.Row([self.btn_jog, self.btn_follow], spacing=8),
                
                ft.Divider(color="#2C2C2E", height=10),
                
                ft.Text("INVIA TARGET FOLLOW", size=11, color="#6B7280", weight=ft.FontWeight.BOLD),
                ft.Row([self.pitch_target_input, self.yaw_target_input], spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Container(
                    content=ft.Text("INVIA TARGET", color="white", size=12, weight=ft.FontWeight.BOLD),
                    bgcolor="#007AFF",
                    border_radius=8,
                    height=35,
                    alignment=ft.Alignment(0, 0),
                    on_click=self.send_follow_target
                )
            ], spacing=6)
        )

        # --- PANNELLO 3: JOG MANUAL CONTROLS (DESTRA) ---
        col_jog_pad = ft.Container(
            expand=3,
            bgcolor="#1C1C1E",
            border_radius=16,
            padding=12,
            content=ft.Column([
                ft.Row([
                    ft.Text("JOG CONTROLS", size=11, color="#6B7280", weight=ft.FontWeight.BOLD),
                    self.jog_speed_label
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                
                ft.Container(
                    content=ft.Column([
                        ft.Row([self.create_jog_arrow(ft.Icons.ARROW_UPWARD_ROUNDED, pitch_speed=1.0)], alignment=ft.MainAxisAlignment.CENTER),
                        ft.Row([
                            self.create_jog_arrow(ft.Icons.ARROW_BACK_ROUNDED, yaw_speed=-1.0),
                            ft.Container(width=42, height=42), 
                            self.create_jog_arrow(ft.Icons.ARROW_FORWARD_ROUNDED, yaw_speed=1.0)
                        ], alignment=ft.MainAxisAlignment.CENTER, spacing=10),
                        ft.Row([self.create_jog_arrow(ft.Icons.ARROW_DOWNWARD_ROUNDED, pitch_speed=-1.0)], alignment=ft.MainAxisAlignment.CENTER)
                    ], spacing=4),
                    alignment=ft.Alignment(0, 0),
                    expand=True
                ),
                
                self.pitch_slider,
                
                ft.Row([
                    ft.Container(
                        content=ft.Text("RESET ERRORE", color="#EF4444", size=11, weight=ft.FontWeight.BOLD),
                        border=ft.Border.all(1, "#EF4444"),
                        border_radius=8,
                        height=28,
                        expand=True,
                        alignment=ft.Alignment(0, 0),
                        on_click=lambda e: self.node.publish_teleop(requested_mode=0, reset_error=True)
                    )
                ])
            ], spacing=4)
        )

        # Configurazione principale della pagina in Landscape
        page.add(
            ft.SafeArea(
                expand=True,
                content=ft.Column([
                    ft.Row([
                        col_telemetry,
                        col_modes,
                        col_jog_pad
                    ], spacing=10, expand=True, vertical_alignment=ft.CrossAxisAlignment.STRETCH),
                    self.flags_text
                ], spacing=4)
            )
        )

        while True:
            await asyncio.sleep(0.15)
            self.refresh_view()
    # --- WIDGET FACTORIES (Stile Tesla) ---
    def create_mode_button(self, icon: str, label: str, mode_id: int):
        # Determina se la modalità necessita dell'esecuzione immediata (Synch=1, Follow=3)
        exec_immediately = mode_id in (1, 3)
        return ft.Container(
            content=ft.Column([
                ft.Icon(icon, color="#6B7280", size=20),
                ft.Text(label, color="#6B7280", size=10, weight=ft.FontWeight.BOLD)
            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, alignment=ft.MainAxisAlignment.CENTER, spacing=2),
            bgcolor="#111111",
            border_radius=10,
            border=ft.Border.all(1, "#2C2C2E"),
            padding=6,
            expand=True,
            height=52,
            on_click=lambda e: self.node.publish_mode(mode_id, execute=exec_immediately)
        )

    def create_jog_arrow(self, icon: str, pitch_speed: float = 0.0, yaw_speed: float = 0.0):
        return ft.GestureDetector(
            on_tap_down=lambda e: self.start_jog(pitch_speed, yaw_speed),
            on_tap_up=lambda e: self.stop_jog(),
            content=ft.Container(
                content=ft.Icon(icon, color="white", size=24),
                width=46,
                height=46,
                alignment=ft.Alignment(0, 0),
                bgcolor="#111111",
                border=ft.Border.all(1, "#374151"),
                border_radius=12,
            ),
        )

    # --- LOGICA DI AGGIORNAMENTO DATI IN TEMPO REALE ---
    def refresh_view(self):
        if self.page is None:
            return
            
        status = self.node.latest_status
        if status is None:
            self.state_text.value = "ATTESA MAN-STATUS..."
            self.page.update()
            return

        pitch_error = status.target_pitch - status.pitch_position
        yaw_error = status.target_yaw - status.yaw_position

        self.state_text.value = f"STATO: {status.state}".upper()
        self.teleop_conn_status.value = f"Teleop: {'ONLINE' if status.teleop_online else 'OFFLINE'}"
        self.teleop_conn_status.color = "#10B981" if status.teleop_online else "#EF4444"
        self.plc_conn_status.value = f"PLC: {'ONLINE' if status.plc_online else 'OFFLINE'}"
        self.plc_conn_status.color = "#10B981" if status.plc_online else "#EF4444"

        self.pitch_pos_text.value = f"PITCH: {status.pitch_position:.2f}°"
        self.yaw_pos_text.value = f"YAW:   {status.yaw_position:.2f}°"
        self.pitch_tgt_text.value = f"TGT: {status.target_pitch:.2f}"
        self.yaw_tgt_text.value = f"TGT: {status.target_yaw:.2f}"
        
        self.error_meta_text.value = f"ΔP: {pitch_error:+.2f}  ΔY: {yaw_error:+.2f}"
        self.flags_text.value = f"Flags: busy={status.busy} | done={status.done} | synch={status.synch} | on_target={status.on_target} | error_active={status.error_active}"

        self._set_error_map(pitch_error, yaw_error)

        # Cambia l'illuminazione in base allo stato attivo
        buttons = [self.btn_idle, self.btn_synch, self.btn_jog, self.btn_follow]
        for idx, btn in enumerate(buttons):
            if status.requested_mode == idx:
                btn.bgcolor = "#1E3A8A"  
                btn.border.color = "#007AFF"
                btn.content.controls[0].color = "#007AFF"  
                btn.content.controls[1].color = "white"     
            else:
                btn.bgcolor = "#111111"
                btn.border.color = "#2C2C2E"
                btn.content.controls[0].color = "#6B7280"  
                btn.content.controls[1].color = "#6B7280"  

        if status.error_active:
            self.badge_text.value = "ERROR"
            self.badge_container.bgcolor = "#7F1D1D"
        elif status.busy:
            self.badge_text.value = "BUSY"
            self.badge_container.bgcolor = "#C2410C"
        else:
            self.badge_text.value = "READY"
            self.badge_container.bgcolor = "#065F46"

        self.page.update()

    def start_jog(self, pitch_speed: float = 0.0, yaw_speed: float = 0.0):
        self.active_jog_pitch = pitch_speed
        self.active_jog_yaw = yaw_speed
        self.publish_jog()

    def stop_jog(self):
        self.active_jog_pitch = 0.0
        self.active_jog_yaw = 0.0
        self.node.publish_teleop(pitch_speed=0.0, yaw_speed=0.0, requested_mode=2)

    def publish_jog(self):
        scaled_pitch = self.active_jog_pitch * self.jog_speed
        scaled_yaw = self.active_jog_yaw * self.jog_speed
        self.node.publish_teleop(pitch_speed=scaled_pitch, yaw_speed=scaled_yaw, requested_mode=2)

    def on_jog_speed_change(self, e):
        self.jog_speed = float(e.control.value or 0.0)
        self.jog_speed_label.value = f"{self.jog_speed:.2f}"
        if self.active_jog_pitch != 0.0 or self.active_jog_yaw != 0.0:
            self.publish_jog()
        if self.page is not None:
            self.page.update()

    def _clamp(self, value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
        return max(minimum, min(maximum, value))

    def _set_error_map(self, pitch_error: float, yaw_error: float):
        pitch_limit = max(0.5, abs(pitch_error))
        yaw_limit = max(0.5, abs(yaw_error))
        x = self._clamp(yaw_error / yaw_limit)
        y = self._clamp(-pitch_error / pitch_limit)
        self.error_target_dot.alignment = ft.Alignment(0, 0)
        self.error_actual_dot.alignment = ft.Alignment(x * 0.82, y * 0.82)

    def send_follow_target(self, e):
        try:
            pitch = float(self.pitch_target_input.value)
            yaw = float(self.yaw_target_input.value)
        except ValueError:
            self.flags_text.value = "Errore: Valori di target non validi!"
            self.page.update()
            return

        self.node.publish_teleop(requested_mode=3, target_pitch=pitch, target_yaw=yaw, send_target=True)
        self.flags_text.value = f"Target Follow inviato con successo: P={pitch:.2f}, Y={yaw:.2f}"
        self.page.update()


# ==========================================
# 3. THREADING & ROS 2 EXECUTION LIFE CYCLE
# ==========================================
def main(args=None):
    rclpy.init(args=args or sys.argv)
    node = ManagerGuiNode()

    def spin_ros():
        rclpy.spin(node)

    def run_gui():
        ft.app(
            target=FletGui(node).main,
            view=ft.AppView.FLET_APP,
        )

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    try:
        run_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
