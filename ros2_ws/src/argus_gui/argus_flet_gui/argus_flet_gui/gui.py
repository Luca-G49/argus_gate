import asyncio
import sys
import threading
from typing import Optional

import flet as ft
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from argus_interfaces.msg import ManagerStatus, PlcCommand, TeleopCommand


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


class FletGui:
    def __init__(self, node: ManagerGuiNode):
        self.node = node
        self.page: Optional[ft.Page] = None
        self.jog_speed = 0.8
        self.active_jog_pitch = 0.0
        self.active_jog_yaw = 0.0
        self.max_history_points = 60
        self.pitch_error_history: list[float] = []
        self.yaw_error_history: list[float] = []
        self.state_text = ft.Text("Waiting for manager status...")
        self.mode_text = ft.Text("")
        self.position_text = ft.Text("")
        self.target_text = ft.Text("")
        self.connection_text = ft.Text("")
        self.error_text = ft.Text("")
        self.error_meta_text = ft.Text("", size=12, color=ft.Colors.BLUE_GREY_700)
        self.error_target_dot = ft.Container(
            expand=True,
            alignment=ft.Alignment(0, 0),
            content=ft.Container(width=10, height=10, border_radius=999, bgcolor=ft.Colors.GREEN_600),
        )
        self.error_actual_dot = ft.Container(
            expand=True,
            alignment=ft.Alignment(0, 0),
            content=ft.Container(width=12, height=12, border_radius=999, bgcolor=ft.Colors.RED_600),
        )
        self.error_box = ft.Container(
            width=220,
            height=120,
            padding=8,
            border_radius=18,
            bgcolor=ft.Colors.BLUE_GREY_100,
            content=ft.Stack([
                self.error_target_dot,
                self.error_actual_dot,
            ]),
        )
        self.badge = ft.Container(
            content=ft.Text("OFFLINE", weight="bold", color=ft.Colors.WHITE),
            padding=ft.Padding(left=12, top=8, right=12, bottom=8),
            border_radius=999,
            bgcolor=ft.Colors.BLUE_GREY_700,
        )
        self.jog_speed_label = ft.Text(f"{self.jog_speed:.2f}", weight="bold")
        self.pitch_slider = ft.Slider(min=0.0, max=1.5, divisions=30, value=self.jog_speed, on_change=self.on_jog_speed_change)
        self.pitch_target = ft.TextField(label="Pitch", width=120, value="0.0")
        self.yaw_target = ft.TextField(label="Yaw", width=120, value="0.0")
        self.log_text = ft.Text("", selectable=True)

    async def main(self, page: ft.Page):
        self.page = page
        page.title = "Argus Gate"
        page.theme_mode = ft.ThemeMode.LIGHT
        page.scroll = ft.ScrollMode.AUTO
        page.padding = 8
        page.bgcolor = ft.Colors.BLUE_GREY_50

        page.add(
            ft.Container(
                padding=10,
                border_radius=16,
                gradient=ft.LinearGradient(
                    begin=ft.Alignment(-1, -1),
                    end=ft.Alignment(1, 1),
                    colors=["#0F172A", "#1E3A8A", "#0EA5E9"],
                ),
                content=ft.Column(
                    [
                        ft.Row(
                            [
                                ft.Text("Argus Gate", size=18, weight="bold", color=ft.Colors.WHITE),
                                self.badge,
                            ],
                            alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                        ),
                        ft.Text("Landscape mobile dashboard", color=ft.Colors.WHITE70, size=11),
                    ],
                    spacing=4,
                ),
            ),
            ft.Container(height=8),
            ft.Row(
                [
                    ft.Container(
                        expand=1,
                        padding=10,
                        border_radius=14,
                        bgcolor=ft.Colors.WHITE,
                        content=ft.Column(
                            [
                                ft.Text("Comandi", size=14, weight="bold"),
                                ft.Row(
                                    [
                                        ft.ElevatedButton("Idle", on_click=lambda e: self.node.publish_mode(0)),
                                        ft.ElevatedButton("Synch", on_click=lambda e: self.node.publish_mode(1, execute=True)),
                                    ],
                                    alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                                ),
                                ft.Row(
                                    [
                                        ft.ElevatedButton("Jog", on_click=lambda e: self.node.publish_mode(2)),
                                        ft.ElevatedButton("Follow", on_click=lambda e: self.node.publish_mode(3, execute=True)),
                                    ],
                                    alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                                ),
                                ft.Container(height=2),
                                ft.Text("Jog", size=12, weight="bold"),
                                ft.Row(
                                    [
                                        self.jog_button("↑", pitch_speed=1.0),
                                        ft.Column(
                                            [
                                                self.jog_button("←", yaw_speed=-1.0),
                                                self.jog_button("→", yaw_speed=1.0),
                                            ],
                                            spacing=8,
                                            horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                                        ),
                                        self.jog_button("↓", pitch_speed=-1.0),
                                    ],
                                    alignment=ft.MainAxisAlignment.SPACE_AROUND,
                                ),
                                ft.Container(height=1),
                                ft.Column(
                                    [
                                        ft.Row(
                                            [
                                                ft.Text("Speed", size=11, weight="bold"),
                                                self.jog_speed_label,
                                            ],
                                            alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                                        ),
                                        self.pitch_slider,
                                    ],
                                    spacing=1,
                                ),
                                ft.Container(height=1),
                                ft.Column(
                                    [
                                        ft.Row(
                                            [self.pitch_target, self.yaw_target],
                                        ),
                                        ft.Row(
                                            [
                                                ft.ElevatedButton("Send target", on_click=self.send_follow_target),
                                                ft.OutlinedButton("Reset error", on_click=lambda e: self.node.publish_teleop(requested_mode=0, reset_error=True)),
                                            ],
                                        ),
                                    ],
                                    spacing=2,
                                ),
                            ],
                            spacing=4,
                        ),
                    ),
                    ft.Container(
                        expand=1,
                        padding=10,
                        border_radius=14,
                        bgcolor=ft.Colors.WHITE,
                        content=ft.Column(
                            [
                                ft.Text("Stato", size=14, weight="bold"),
                                ft.Container(content=self.state_text, padding=6, bgcolor=ft.Colors.BLUE_GREY_50, border_radius=10),
                                ft.Container(content=self.mode_text, padding=6, bgcolor=ft.Colors.BLUE_GREY_50, border_radius=10),
                                self.connection_text,
                                self.position_text,
                                self.target_text,
                                ft.Column([self.error_text, self.error_meta_text], spacing=0),
                                ft.Container(
                                    padding=8,
                                    border_radius=12,
                                    bgcolor=ft.Colors.BLUE_GREY_50,
                                    content=ft.Column(
                                        [
                                            ft.Row(
                                                [
                                                    ft.Text("Errore", weight="bold"),
                                                    ft.Text("verde=zero, rosso=errore", size=11, color=ft.Colors.BLUE_GREY_600),
                                                ],
                                                alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                                            ),
                                            self.error_box,
                                        ],
                                        spacing=4,
                                    ),
                                ),
                                self.log_text,
                            ],
                            spacing=4,
                        ),
                    ),
                ],
                spacing=8,
                alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                vertical_alignment=ft.CrossAxisAlignment.START,
            ),
        )

        page.update()

        while True:
            await asyncio.sleep(0.2)
            self.refresh_view()

    def refresh_view(self):
        if self.page is None:
            return
        status = self.node.latest_status
        if status is None:
            self.state_text.value = "Waiting for manager status..."
            self.mode_text.value = ""
            self.connection_text.value = "Teleop: -- | PLC: --"
            self.position_text.value = "Position: --"
            self.target_text.value = "Target: --"
            self.error_text.value = "Error tracking: --"
            self.error_meta_text.value = ""
            self._set_error_map(0.0, 0.0)
            self.log_text.value = ""
            self.page.update()
            return

        pitch_error = status.target_pitch - status.pitch_position
        yaw_error = status.target_yaw - status.yaw_position

        self.state_text.value = f"State: {status.state}"
        self.mode_text.value = f"Requested mode: {status.requested_mode}"
        self.connection_text.value = (
            f"Teleop: {'online' if status.teleop_online else 'offline'} | "
            f"PLC: {'online' if status.plc_online else 'offline'}"
        )
        self.position_text.value = f"Position: pitch={status.pitch_position:.2f} yaw={status.yaw_position:.2f}"
        self.target_text.value = f"Target: pitch={status.target_pitch:.2f} yaw={status.target_yaw:.2f}"
        self.error_text.value = f"Tracking error: pitch={pitch_error:.2f} yaw={yaw_error:.2f}"
        self.error_meta_text.value = f"pitch {pitch_error:+.2f}   yaw {yaw_error:+.2f}"
        self.log_text.value = (
            f"busy={status.busy} done={status.done} synch={status.synch} on_target={status.on_target} "
            f"error={status.error_active}"
        )

        self._set_error_map(pitch_error, yaw_error)

        if status.error_active:
            self.badge.content = ft.Text("ERROR", weight="bold", color=ft.Colors.WHITE)
            self.badge.bgcolor = ft.Colors.RED_700
        elif status.busy:
            self.badge.content = ft.Text("BUSY", weight="bold", color=ft.Colors.WHITE)
            self.badge.bgcolor = ft.Colors.ORANGE_700
        else:
            self.badge.content = ft.Text("READY", weight="bold", color=ft.Colors.WHITE)
            self.badge.bgcolor = ft.Colors.GREEN_700
        self.page.update()

    def jog_button(self, label: str, pitch_speed: float = 0.0, yaw_speed: float = 0.0):
        return ft.GestureDetector(
            on_tap_down=lambda e: self.start_jog(pitch_speed, yaw_speed),
            on_tap_up=lambda e: self.stop_jog(),
            content=ft.Container(
                content=ft.Text(label, size=24, weight="bold"),
                width=60,
                height=60,
                alignment=ft.Alignment.CENTER,
                bgcolor=ft.Colors.BLUE_100,
                border_radius=18,
            ),
        )

    def start_jog(self, pitch_speed: float = 0.0, yaw_speed: float = 0.0):
        self.active_jog_pitch = pitch_speed
        self.active_jog_yaw = yaw_speed
        self.publish_jog()

    def stop_jog(self):
        self.active_jog_pitch = 0.0
        self.active_jog_yaw = 0.0
        self.node.publish_teleop(
            pitch_speed=0.0,
            yaw_speed=0.0,
            requested_mode=2,
        )

    def publish_jog(self):
        scaled_pitch = self.active_jog_pitch * self.jog_speed
        scaled_yaw = self.active_jog_yaw * self.jog_speed
        self.node.publish_teleop(
            pitch_speed=scaled_pitch,
            yaw_speed=scaled_yaw,
            requested_mode=2,
        )

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
            pitch = float(self.pitch_target.value)
            yaw = float(self.yaw_target.value)
        except ValueError:
            self.log_text.value = "Invalid target values"
            self.page.update()
            return

        self.node.publish_teleop(
            requested_mode=3,
            target_pitch=pitch,
            target_yaw=yaw,
            send_target=True,
        )
        self.log_text.value = f"Follow target sent: pitch={pitch:.2f} yaw={yaw:.2f}"
        self.page.update()


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
