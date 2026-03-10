#!/usr/bin/env python3
import os
import select
import termios
import threading
import time
import tty

import rcl_interfaces.msg
import rclpy

from .keyboard_teleop_common import (
    COMMON_KEYBOARD_MESSAGE,
    create_core_from_params,
    create_twist_publisher_adapter,
    declare_common_parameters,
)


class KeyboardTtyTeleopNode:
    def __init__(self):
        self.node = rclpy.create_node("keyboard_tty_teleop_node")
        self.common_params = declare_common_parameters(self.node)
        read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        self.repeat_timeout_sec = float(
            self.node.declare_parameter(
                "repeat_timeout_sec",
                0.35,
                read_only_descriptor,
            ).value
        )
        self.read_poll_timeout = float(
            self.node.declare_parameter(
                "read_poll_timeout",
                0.02,
                read_only_descriptor,
            ).value
        )
        self.tty_device_path = str(
            self.node.declare_parameter(
                "tty_device_path",
                "",
                read_only_descriptor,
            ).value
        )
        self.core = create_core_from_params(self.common_params)
        self.publisher = create_twist_publisher_adapter(self.node, self.common_params)
        self.stop_event = threading.Event()
        publish_period = 1.0 / max(1.0, float(self.common_params["publish_rate"]))
        self.node.create_timer(publish_period, self._publish_timer_callback)

    def _publish_timer_callback(self) -> None:
        command = self.core.snapshot(time.monotonic(), stale_key_timeout=self.repeat_timeout_sec)
        self.publisher.publish(command)

    def handle_input_char(self, char: str, now: float | None = None) -> bool:
        timestamp = time.monotonic() if now is None else float(now)
        if char == "\x03":
            self.stop_event.set()
            if rclpy.ok():
                rclpy.shutdown()
            return False
        normalized = char.lower()
        return self.core.handle_key_press(normalized, timestamp)

    def shutdown(self) -> None:
        self.stop_event.set()
        self.core.emergency_stop(time.monotonic())
        if rclpy.ok():
            self.publisher.publish_zero(self.core)
        self.node.destroy_node()


class TtyInputThread(threading.Thread):
    def __init__(self, teleop: KeyboardTtyTeleopNode):
        super().__init__(daemon=True)
        self.teleop = teleop
        self._tty_fd = None
        self._old_termios = None

    def run(self) -> None:
        try:
            tty_path = self._resolve_tty_path()
            self._tty_fd = os.open(tty_path, os.O_RDONLY | os.O_NONBLOCK)
            self._old_termios = termios.tcgetattr(self._tty_fd)
            tty.setraw(self._tty_fd)
            self.teleop.node.get_logger().info(f"keyboard tty teleop attached to {tty_path}")
        except OSError:
            self.teleop.node.get_logger().error("keyboard tty teleop 无法打开可交互终端，需要从交互式终端启动")
            self.teleop.stop_event.set()
            if rclpy.ok():
                rclpy.shutdown()
            return

        try:
            while rclpy.ok() and not self.teleop.stop_event.is_set():
                ready_fds, _, _ = select.select([self._tty_fd], [], [], self.teleop.read_poll_timeout)
                if not ready_fds:
                    continue
                chunk = os.read(self._tty_fd, 64)
                if not chunk:
                    continue
                timestamp = time.monotonic()
                for byte in chunk:
                    self.teleop.handle_input_char(chr(byte), timestamp)
        finally:
            if self._tty_fd is not None and self._old_termios is not None:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._old_termios)
            if self._tty_fd is not None:
                os.close(self._tty_fd)

    def _resolve_tty_path(self) -> str:
        candidates = []
        explicit_path = self.teleop.tty_device_path.strip()
        if explicit_path:
            candidates.append(explicit_path)
        try:
            candidates.append(os.ttyname(0))
        except OSError:
            pass
        candidates.append("/dev/tty")

        for candidate in candidates:
            if not candidate:
                continue
            try:
                fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                continue
            try:
                if os.isatty(fd):
                    return candidate
            finally:
                os.close(fd)

        raise OSError("no interactive tty available")


def main():
    print(COMMON_KEYBOARD_MESSAGE)
    print("TTY 模式通过按键重复推断松键；退出请按 Ctrl-C")

    rclpy.init()
    teleop = KeyboardTtyTeleopNode()
    input_thread = TtyInputThread(teleop)
    input_thread.start()
    try:
        rclpy.spin(teleop.node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        input_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
