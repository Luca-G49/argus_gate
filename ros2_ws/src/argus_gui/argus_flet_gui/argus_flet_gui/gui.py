import asyncio
import sys
import threading
from typing import Optional

import flet as ft
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from argus_interfaces.msg import ManagerStatus, PlcCommand, TeleopCommand


# ==============================================================================
# 1. ROS 2 MIDDLEWARE ENGINE (Decoupled Background Thread Operations)
# ==============================================================================
class ManagerGuiNode(Node):
    def __init__(self):
        super().__init__('argus_flet_gui_node')
        
        # Configure best-effort QoS profile for high-frequency telemetry data streams
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.declare_parameter('publish_rate_hz', 50.0)
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Initialize ROS 2 Pub/Sub interfaces
        self.status_sub = self.create_subscription(ManagerStatus, 'manager_status', self.status_cb, qos)
        self.plc_cmd_pub = self.create_publisher(PlcCommand, 'plc_command', 10)
        self.teleop_pub = self.create_publisher(TeleopCommand, 'teleop_cmd', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_loop)

        # Thread-safe interface register memory layout
        self.latest_status: Optional[ManagerStatus] = None
        self.pitch_speed = 0.0
        self.yaw_speed = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.requested_mode = 0
        self.execute_action = False
        self.reset_error = False
        self.send_target = False
        self.fire_cmd = False 

    def status_cb(self, msg: ManagerStatus):
        """Asynchronous incoming telemetry buffer callback."""
        self.latest_status = msg

    def publish_mode(self, mode: int, execute: bool = False):
        """Dispatches field state machine switches to the master controller."""
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
                       reset_error: bool = False,
                       fire: bool = False):
        """Synchronizes registers to decouple background loop data bindings."""
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed
        self.requested_mode = requested_mode
        self.target_pitch = target_pitch
        self.target_yaw = target_yaw
        self.send_target = send_target
        self.execute_action = execute_action or fire 
        self.reset_error = reset_error
        self.fire_cmd = fire

    def publish_loop(self):
        """High-frequency deterministic transmission loop mapped to ROS 2 clock."""
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

        # Volatile flag automated software clearing sequence
        self.execute_action = False
        self.reset_error = False
        self.send_target = False
        self.fire_cmd = False
# ==============================================================================
# 2. FLET COMPONENT COMPOSITION (Atomic Widget Instantiation)
# ==============================================================================
class FletGui:
    def __init__(self, node: ManagerGuiNode):
        self.node = node
        self.page: Optional[ft.Page] = None
        self.jog_speed = 0.8
        self.active_jog_pitch = 0.0
        self.active_jog_yaw = 0.0

        # UI Components - Left Grid Data Displays
        self.state_text = ft.Text("ATTESA TELEMETRIA...", color="white", size=14, weight=ft.FontWeight.BOLD)
        self.teleop_conn_status = ft.Text("Teleop: --", color="#6B7280", size=11)
        self.plc_conn_status = ft.Text("PLC: --", color="#6B7280", size=11)
        
        self.pitch_pos_text = ft.Text("PITCH: --", color="#9CA3AF", size=12, weight=ft.FontWeight.W_500)
        self.yaw_pos_text = ft.Text("YAW: --", color="#9CA3AF", size=12, weight=ft.FontWeight.W_500)
        self.pitch_tgt_text = ft.Text("TARGET: --", color="#6B7280", size=11)
        self.yaw_tgt_text = ft.Text("TARGET: --", color="#6B7280", size=11)
        
        self.badge_text = ft.Text("OFFLINE", weight="bold", color="white", size=11)
        self.badge_container = ft.Container(
            content=self.badge_text, padding=ft.Padding(left=12, top=6, right=12, bottom=6),
            border_radius=8, bgcolor="#374151",
        )

        # UI Components - HUD Alignment Tracking Dot Overlays
        self.error_target_dot = ft.Container(
            expand=True, alignment=ft.Alignment(0, 0),
            content=ft.Container(width=8, height=8, border_radius=999, bgcolor="#10B981"),
        )
        self.error_actual_dot = ft.Container(
            expand=True, alignment=ft.Alignment(0, 0),
            content=ft.Container(width=10, height=10, border_radius=999, bgcolor="#EF4444"),
        )
        self.error_reticle = ft.Container(
            expand=True, alignment=ft.Alignment(0, 0),
            content=ft.Container(width=24, height=24, border_radius=999, border=ft.Border.all(1, "transparent"))
        )
        self.error_box = ft.Container(
            width=120, height=120, border_radius=12, bgcolor="#111111", border=ft.Border.all(1, "#374151"),
            content=ft.Stack([self.error_target_dot, self.error_actual_dot, self.error_reticle]),
        )
        self.error_meta_text = ft.Text("ΔP: 0.00  ΔY: 0.00", size=11, color="#6B7280", font_family="monospace")

        # UI Components - Operational Grid State Switches
        self.btn_idle = self.create_mode_button(ft.Icons.PAUSE_ROUNDED, "IDLE", 0)
        self.btn_synch = self.create_mode_button(ft.Icons.SYNC_ALT_ROUNDED, "SYNCH", 1)
        self.btn_jog = self.create_mode_button(ft.Icons.PAN_TOOL_ALT_ROUNDED, "JOG", 2)
        self.btn_follow = self.create_mode_button(ft.Icons.TRACK_CHANGES_ROUNDED, "FOLLOW", 3)

        # UI Components - Closed Loop Vector Coordinates Fields
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

        # UI Components - Permanent Active Tactical Weapon Trigger (Always Armed)
        self.btn_fire = ft.Container(
            content=ft.Text("FIRE NOW", color="white", size=11, weight=ft.FontWeight.BOLD),
            bgcolor="#EF4444", 
            border_radius=8, height=30, alignment=ft.Alignment(0, 0),
            on_click=self.trigger_fire_command, disabled=False, opacity=1.0
        )

        # UI Components - Feedrate Calibration Ingestion
        self.jog_speed_label = ft.Text(f"{self.jog_speed:.2f}", weight="bold", color="white", size=12)
        self.pitch_slider = ft.Slider(
            min=0.0, max=1.5, divisions=30, value=self.jog_speed,
            active_color="#007AFF", inactive_color="#374151", on_change=self.on_jog_speed_change
        )
        self.flags_text = ft.Text("Flags: --", size=10, color="#6B7280", overflow=ft.TextOverflow.ELLIPSIS)
# ==============================================================================
# 3. LAYOUT COMPOSITION MATRIX (Landscape Viewport Window Segmentation)
# ==============================================================================
    async def main(self, page: ft.Page):
        self.page = page
        page.title = "Argus Gate Mobile Dashboard"
        page.theme_mode = ft.ThemeMode.DARK
        page.bgcolor = "#111111"
        page.padding = 10
        
        # Enforce exact landscape smartphone bounds configurations
        page.window.width = 844
        page.window.height = 390
        page.window.resizable = False

        # --- GRID CELL 1: FEEDBACK MATRIX (LEFT SECTION) ---
        col_telemetry = ft.Container(
            expand=4, bgcolor="#1C1C1E", border_radius=16, padding=12,
            content=ft.Column([
                ft.Row([
                    ft.Column([ft.Text("ARGUS GATE", size=16, weight=ft.FontWeight.BOLD, color="white"), self.state_text], spacing=2),
                    self.badge_container
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                ft.Divider(color="#2C2C2E", height=10),
                ft.Row([
                    ft.Column([
                        ft.Text("CURRENT POSITION", size=10, color="#6B7280", weight=ft.FontWeight.BOLD),
                        self.pitch_pos_text, self.yaw_pos_text, ft.Container(height=4),
                        ft.Text("CONNECTIONS", size=10, color="#6B7280", weight=ft.FontWeight.BOLD),
                        self.teleop_conn_status, self.plc_conn_status,
                    ], spacing=2, expand=True),
                    ft.Column([self.error_box, ft.Container(content=self.error_meta_text, alignment=ft.Alignment(0, 0))], spacing=4, horizontal_alignment=ft.CrossAxisAlignment.CENTER)
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN, vertical_alignment=ft.CrossAxisAlignment.CENTER),
            ], spacing=4)
        )

        # --- GRID CELL 2: STATE TRIGGERS GRID (CENTER SECTION) ---
        col_modes = ft.Container(
            expand=3, bgcolor="#1C1C1E", border_radius=16, padding=12,
            content=ft.Column([
                ft.Text("OPERATIONAL MODES", size=11, color="#6B7280", weight=ft.FontWeight.BOLD),
                ft.Row([self.btn_idle, self.btn_synch], spacing=8),
                ft.Row([self.btn_jog, self.btn_follow], spacing=8),
                ft.Divider(color="#2C2C2E", height=10),
                ft.Text("TRANSMIT CLOSED-LOOP TARGET", size=11, color="#6B7280", weight=ft.FontWeight.BOLD),
                ft.Row([self.pitch_target_input, self.yaw_target_input], spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Container(
                    content=ft.Text("SEND TARGET", color="white", size=12, weight=ft.FontWeight.BOLD),
                    bgcolor="#007AFF", border_radius=8, height=35, alignment=ft.Alignment(0, 0), on_click=self.send_follow_target
                )
            ], spacing=6)
        )

        # --- GRID CELL 3: TELEOPERATION INTERFACE (RIGHT SECTION) ---
        col_jog_pad = ft.Container(
            expand=3, bgcolor="#1C1C1E", border_radius=16, padding=12,
            content=ft.Column([
                ft.Row([ft.Text("JOG CONTROLS", size=11, color="#6B7280", weight=ft.FontWeight.BOLD), self.jog_speed_label], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
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
                    alignment=ft.Alignment(0, 0), expand=True
                ),
                ft.Container(height=2),
                self.btn_fire, 
                ft.Container(height=2),
                self.pitch_slider,
                ft.Row([
                    ft.Container(
                        content=ft.Text("RESET FAULT", color="#EF4444", size=11, weight=ft.FontWeight.BOLD),
                        border=ft.Border.all(1, "#EF4444"), border_radius=8, height=28, expand=True, alignment=ft.Alignment(0, 0),
                        on_click=lambda e: self.node.publish_teleop(requested_mode=0, reset_error=True)
                    )
                ])
            ], spacing=2) 
        )

        # Render application main multi-grid horizontal structure
        page.add(
            ft.SafeArea(
                expand=True,
                content=ft.Column([
                    ft.Row([col_telemetry, col_modes, col_jog_pad], spacing=10, expand=True, vertical_alignment=ft.CrossAxisAlignment.STRETCH),
                    self.flags_text
                ], spacing=4)
            )
        )

        # Thread blocker loop drives presentation layer pipelines updates
        while True:
            await asyncio.sleep(0.15)
            self.refresh_view()
    # --- WIDGET ASSEMBLY METHODS (Decoupled Factory Layout Patterns) ---
    def create_mode_button(self, icon: str, label: str, mode_id: int):
        exec_immediately = mode_id in (1, 3)
        return ft.Container(
            content=ft.Column([
                ft.Icon(icon, color="#6B7280", size=20),
                ft.Text(label, color="#6B7280", size=10, weight=ft.FontWeight.BOLD)
            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, alignment=ft.MainAxisAlignment.CENTER, spacing=2),
            bgcolor="#111111", border_radius=10, border=ft.Border.all(1, "#2C2C2E"), padding=6, expand=True, height=52,
            on_click=lambda e: self.node.publish_mode(mode_id, execute=exec_immediately)
        )

    def create_jog_arrow(self, icon: str, pitch_speed: float = 0.0, yaw_speed: float = 0.0):
        return ft.GestureDetector(
            on_tap_down=lambda e: self.start_jog(pitch_speed, yaw_speed),
            on_tap_up=lambda e: self.stop_jog(),
            content=ft.Container(
                content=ft.Icon(icon, color="white", size=24), width=46, height=46, alignment=ft.Alignment(0, 0),
                bgcolor="#111111", border=ft.Border.all(1, "#374151"), border_radius=12,
            ),
        )

    def trigger_fire_command(self, e):
        """Asynchronously dispatches the permanently armed fire register state data."""
        self.node.publish_teleop(execute_action=True, fire=True)
        self.flags_text.value = "HUD ALERT: TACTICAL WEAPON DISCHARGE SEQUENCE EXECUTED!"
        self.page.update()

    # --- TELEMETRY GRAPHICAL SYNCHRONIZATION RUNTIME LOOP ---
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

        # --- HUD COORDINATES RENDERING CHANNELS (REMOVED DYNAMIC COLORING) ---
        # Coordinate values remain locked to a professional consistent gray spectrum
        self.pitch_pos_text.value = f"PITCH: {status.pitch_position:.2f}°"
        self.pitch_pos_text.color = "#9CA3AF"

        self.yaw_pos_text.value = f"YAW:   {status.yaw_position:.2f}°"
        self.yaw_pos_text.color = "#9CA3AF"
        
        # --- HUD LOCK TARGET RETICLE OVERLAY INDICATOR ---
        if status.on_target:
            self.error_reticle.content.border = ft.Border.all(1.5, "#10B981") # Glow green ring
        else:
            self.error_reticle.content.border = ft.Border.all(1, "transparent") # Hide lock ring
        
        self.pitch_tgt_text.value = f"TGT: {status.target_pitch:.2f}"
        self.yaw_tgt_text.value = f"TGT: {status.target_yaw:.2f}"
        self.error_meta_text.value = f"ΔP: {pitch_error:+.2f}  ΔY: {yaw_error:+.2f}"
        self.flags_text.value = f"Flags: busy={status.busy} | done={status.done} | synch={status.synch} | on_target={status.on_target} | error_active={status.error_active}"

        self._set_error_map(pitch_error, yaw_error)

        # --- COMPONENT OPERATIONAL MODE STATE RE-RENDERING PIPELINES ---
        ros_mode = str(status.requested_mode).strip().upper()
        mode_mapping = {"0": 0, "1": 1, "2": 2, "3": 3, "IDLE": 0, "SYNCH": 1, "JOG": 2, "FOLLOW": 3}
        active_idx = mode_mapping.get(ros_mode, -1)

        buttons = [self.btn_idle, self.btn_synch, self.btn_jog, self.btn_follow]
        for idx, btn in enumerate(buttons):
            if not btn.content or not hasattr(btn.content, 'controls') or len(btn.content.controls) < 2:
                continue
            if active_idx == idx:
                btn.bgcolor = "#1E3A8A"  
                btn.border = ft.Border.all(1, "#007AFF")       
                btn.content.controls[0].color = "#007AFF" 
                btn.content.controls[1].color = "white"    
            else:
                btn.bgcolor = "#111111"  
                btn.border = ft.Border.all(1, "#2C2C2E")          
                btn.content.controls[0].color = "#6B7280" 
                btn.content.controls[1].color = "#6B7280" 

        # Update Master Core System Badges
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
        pitch_limit = max(90.0, abs(pitch_error))
        yaw_limit = max(90.0, abs(yaw_error))
        x = self._clamp(yaw_error / yaw_limit)
        y = self._clamp(-pitch_error / pitch_limit)
        self.error_target_dot.alignment = ft.Alignment(0, 0)
        self.error_actual_dot.alignment = ft.Alignment(x * 0.82, y * 0.82)

    def send_follow_target(self, e):
        try:
            pitch = float(self.pitch_target_input.value)
            yaw = float(self.yaw_target_input.value)
        except ValueError:
            self.flags_text.value = "Error: Invalid numeric formatting inside input target fields!"
            self.page.update()
            return
        self.node.publish_teleop(requested_mode=3, target_pitch=pitch, target_yaw=yaw, send_target=True)
        self.flags_text.value = f"Closed-loop follow target transmitted successfully: P={pitch:.2f}, Y={yaw:.2f}"
        self.page.update()
# ==============================================================================
# 5. MULTI-THREAD LIFECYCLE MANAGEMENT (Application Thread Orchestration Entry)
# ==============================================================================
def main(args=None):
    # Initialize global messaging framework middleware context
    rclpy.init(args=args or sys.argv)
    node = ManagerGuiNode()

    def spin_ros():
        """Worker background loop processing incoming ROS subscription message handlers."""
        try:
            rclpy.spin(node)
        except SystemExit:
            pass

    def run_gui():
        """Worker lifecycle thread initializing front-end framework viewport rendering."""
        ft.app(
            target=FletGui(node).main,
            view=ft.AppView.FLET_APP,
        )

    # Detach ROS executor queue pipelines to keep GUI interactions fully responsive and atomic
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    try:
        run_gui()
    finally:
        # Guarantee predictable destruction lifecycle sequencing for software resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
