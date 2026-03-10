#!/usr/bin/env python3
import threading
import time

import rcl_interfaces.msg
import rclpy

from .keyboard_teleop_common import (
    COMMON_KEYBOARD_MESSAGE,
    create_core_from_params,
    create_twist_publisher_adapter,
    declare_common_parameters,
)


class KeyboardGuiTeleopNode:
    def __init__(self):
        self.node = rclpy.create_node("keyboard_gui_teleop_node")
        self.common_params = declare_common_parameters(self.node)
        read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        self.window_title = str(
            self.node.declare_parameter(
                "window_title",
                "UGV Keyboard Teleop",
                read_only_descriptor,
            ).value
        )
        self.focus_loss_zero_cmd = bool(
            self.node.declare_parameter(
                "focus_loss_zero_cmd",
                True,
                read_only_descriptor,
            ).value
        )
        self.core = create_core_from_params(self.common_params)
        self.publisher = create_twist_publisher_adapter(self.node, self.common_params)
        self._status_lock = threading.Lock()
        self._status_text = ""
        self._pressed_keys: set[str] = set()

        publish_period = 1.0 / max(1.0, float(self.common_params["publish_rate"]))
        self.node.create_timer(publish_period, self._publish_timer_callback)

    def _publish_timer_callback(self) -> None:
        command = self.core.snapshot(time.monotonic())
        self.publisher.publish(command)
        self._update_status_text(command)

    def _update_status_text(self, command) -> None:
        active_keys = "".join(command.active_keys) if command.active_keys else "-"
        text = (
            f"{COMMON_KEYBOARD_MESSAGE.strip()}\n\n"
            f"当前速度: 线速度 {command.linear_speed:.2f} m/s | 角速度 {command.angular_speed:.2f} rad/s\n"
            f"当前按键: {active_keys}\n"
            f"窗口失焦会立即停车"
        )
        with self._status_lock:
            self._status_text = text

    def get_status_text(self) -> str:
        with self._status_lock:
            return self._status_text

    def handle_key_press(self, key: str) -> None:
        if not key:
            return
        normalized = key.lower()
        if normalized in self._pressed_keys:
            return
        if self.core.handle_key_press(normalized, time.monotonic()):
            self._pressed_keys.add(normalized)

    def handle_key_release(self, key: str) -> None:
        if not key:
            return
        normalized = key.lower()
        self._pressed_keys.discard(normalized)
        self.core.handle_key_release(normalized, time.monotonic())

    def handle_focus_lost(self) -> None:
        self._pressed_keys.clear()
        self.core.clear_move_keys(time.monotonic())
        if self.focus_loss_zero_cmd:
            self.publisher.publish_zero(self.core)

    def shutdown(self) -> None:
        self.core.emergency_stop(time.monotonic())
        self.publisher.publish_zero(self.core)
        self.node.destroy_node()


def main():
    try:
        import tkinter as tk
    except ImportError:
        print("tkinter 不可用，请改用 keyboard_backend:=tty")
        return

    rclpy.init()
    teleop = KeyboardGuiTeleopNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleop.node,), daemon=True)
    spinner.start()

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        teleop.node.get_logger().error(f"无法启动键盘 GUI teleop: {exc}")
        teleop.shutdown()
        rclpy.shutdown()
        spinner.join(timeout=1.0)
        return

    status_var = tk.StringVar(value=COMMON_KEYBOARD_MESSAGE.strip())

    def on_key_press(event):
        teleop.handle_key_press(event.char or event.keysym)

    def on_key_release(event):
        teleop.handle_key_release(event.char or event.keysym)

    def on_focus_out(_event):
        teleop.handle_focus_lost()

    def refresh_status():
        status_var.set(teleop.get_status_text())
        if root.winfo_exists():
            root.after(100, refresh_status)

    def close_window():
        teleop.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.title(teleop.window_title)
    root.geometry("460x280")
    root.resizable(False, False)
    root.bind("<KeyPress>", on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.bind("<FocusOut>", on_focus_out)
    root.protocol("WM_DELETE_WINDOW", close_window)
    root.configure(padx=16, pady=16)

    label = tk.Label(
        root,
        textvariable=status_var,
        justify="left",
        anchor="nw",
        font=("TkFixedFont", 11),
    )
    label.pack(fill="both", expand=True)

    root.focus_force()
    refresh_status()
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            teleop.shutdown()
            rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == "__main__":
    main()
