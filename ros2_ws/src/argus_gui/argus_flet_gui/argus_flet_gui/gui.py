import asyncio
import sys
import threading
from typing import Optional

import math

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

        self.status_sub = self.create_subscription(ManagerStatus, 'manager_status', self.status_cb, qos)
        self.plc_cmd_pub = self.create_publisher(PlcCommand, 'plc_command', 10)
        self.teleop_pub = self.create_publisher(TeleopCommand, 'teleop_cmd', 10)
        self.latest_status: Optional[ManagerStatus] = None

    def status_cb(self, msg: ManagerStatus):
        self.latest_status = msg

    def publish_mode(self, mode: int, execute: bool = False):
        cmd = PlcCommand()
        cmd.mode = mode
        cmd.exec = execute
        self.plc_cmd_pub.publish(cmd)

    def publish_teleop(self,
                       pitch_speed: float = 0.0,
                       yaw_speed: float = 0.0,
                       requested_mode: int = 2,
                       target_pitch: float = 0.0,
                       target_yaw: float = 0.0,
                       send_target: bool = False,
                       execute_action: bool = False,
                       reset_error: bool = False):
        msg = TeleopCommand()
        msg.pitch_speed = pitch_speed
        msg.yaw_speed = yaw_speed
        msg.requested_mode = requested_mode
        msg.target_pitch = target_pitch
        msg.target_yaw = target_yaw
        msg.send_target = send_target
        msg.execute_action = execute_action
        msg.reset_error = reset_error
        self.teleop_pub.publish(msg)


class FletGui:
    def __init__(self, node: ManagerGuiNode):
        self.node = node
        self.page: Optional[ft.Page] = None
        self.state_text = ft.Text("Waiting for manager status...")
        self.mode_text = ft.Text("")
        self.position_text = ft.Text("")
        self.target_text = ft.Text("")
        self.badge = ft.Container(content=ft.Text("OFFLINE"), padding=10, border_radius=8)
        self.pitch_slider = ft.Slider(min=-1.0, max=1.0, divisions=20, value=0.0)
        self.yaw_slider = ft.Slider(min=-1.0, max=1.0, divisions=20, value=0.0)
        self.pitch_target = ft.TextField(label="Pitch target", value="0.0")
        self.yaw_target = ft.TextField(label="Yaw target", value="0.0")
        self.log_text = ft.Text("", selectable=True)

    async def main(self, page: ft.Page):
        self.page = page
        page.title = "Argus Gate"
        page.theme_mode = ft.ThemeMode.LIGHT
        page.vertical_alignment = ft.MainAxisAlignment.START
        page.horizontal_alignment = ft.CrossAxisAlignment.START

        page.add(
            ft.Text("Argus Gate Manager", size=24, weight="bold"),
            ft.Row([
                ft.ElevatedButton("Idle", on_click=lambda e: self.node.publish_mode(0)),
                ft.ElevatedButton("Synch", on_click=lambda e: self.node.publish_mode(1, execute=True)),
                ft.ElevatedButton("Jog", on_click=lambda e: self.node.publish_mode(2)),
                ft.ElevatedButton("Follow", on_click=lambda e: self.node.publish_mode(3, execute=True)),
            ]),
            ft.Row([
                self.jog_button("↑", pitch_speed=0.8),
                ft.Column([
                    self.jog_button("←", yaw_speed=-0.8),
                    self.jog_button("→", yaw_speed=0.8),
                ], alignment=ft.MainAxisAlignment.CENTER),
                self.jog_button("↓", pitch_speed=-0.8),
            ]),
            ft.Row([
                ft.Text("Pitch speed"),
                self.pitch_slider,
                ft.Text("Yaw speed"),
                self.yaw_slider,
            ]),
            ft.Row([
                self.pitch_target,
                self.yaw_target,
                ft.ElevatedButton("Send target", on_click=self.send_follow_target),
                ft.ElevatedButton("Reset error", on_click=lambda e: self.node.publish_teleop(requested_mode=0, reset_error=True)),
            ]),
            ft.Container(
                content=ft.Column([
                    ft.Text("Manager status"),
                    self.state_text,
                    self.mode_text,
                    self.position_text,
                    self.target_text,
                    self.badge,
                ]),
                padding=10,
                border=ft.border.all(1, ft.Colors.GREY_300),
                border_radius=8,
            ),
            ft.Container(
                content=ft.Column([ft.Text("Events"), self.log_text]),
                padding=10,
                border=ft.border.all(1, ft.Colors.GREY_300),
                border_radius=8,
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
            self.page.update()
            return

        self.state_text.value = f"State: {status.state}"
        self.mode_text.value = f"Requested mode: {status.requested_mode}"
        self.position_text.value = f"Position: pitch={status.pitch_position:.2f} yaw={status.yaw_position:.2f}"
        self.target_text.value = f"Target: pitch={status.target_pitch:.2f} yaw={status.target_yaw:.2f}"
        self.log_text.value = (
            f"busy={status.busy} done={status.done} synch={status.synch} on_target={status.on_target} "
            f"error={status.error_active}"
        )

        if status.error_active:
            self.badge.content = ft.Text("ERROR", color=ft.Colors.RED_700)
        elif status.busy:
            self.badge.content = ft.Text("BUSY", color=ft.Colors.ORANGE_700)
        else:
            self.badge.content = ft.Text("READY", color=ft.Colors.GREEN_700)
        self.page.update()

    def jog_button(self, label: str, pitch_speed: float = 0.0, yaw_speed: float = 0.0):
        return ft.GestureDetector(
            on_pointer_down=lambda e: self.node.publish_teleop(
                pitch_speed=pitch_speed,
                yaw_speed=yaw_speed,
                requested_mode=2,
            ),
            on_pointer_up=lambda e: self.node.publish_teleop(
                pitch_speed=0.0,
                yaw_speed=0.0,
                requested_mode=2,
            ),
            on_pointer_cancel=lambda e: self.node.publish_teleop(
                pitch_speed=0.0,
                yaw_speed=0.0,
                requested_mode=2,
            ),
            content=ft.Container(
                content=ft.Text(label, size=24, weight="bold"),
                width=70,
                height=70,
                alignment=ft.Alignment.CENTER,
                bgcolor=ft.Colors.BLUE_100,
                border=ft.border.all(1, ft.Colors.BLUE_GREY_300),
                border_radius=10,
            ),
        )

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
