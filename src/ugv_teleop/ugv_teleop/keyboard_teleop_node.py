#!/usr/bin/env python3
import fcntl
import glob
import os
import select
import struct
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
from .keyboard_teleop_core import MOVE_BINDINGS, SPEED_BINDINGS


EV_KEY = 0x01
KEY_LEFTCTRL = 29
KEY_RIGHTCTRL = 97
KEY_C = 46

KEY_CODE_TO_CHAR = {
    16: "q",
    17: "w",
    18: "e",
    23: "i",
    24: "o",
    30: "a",
    31: "s",
    32: "d",
    37: "k",
    38: "l",
    45: "x",
}

EVIOCGRAB = 0x40044590


def discover_evdev_keyboard_devices(explicit_device):
    if explicit_device:
        if os.path.exists(explicit_device):
            return [explicit_device]
        return []

    devices = sorted(glob.glob('/dev/input/by-path/*-event-kbd'))
    if devices:
        return devices

    fallback = []
    try:
        with open('/proc/bus/input/devices', 'r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line.startswith('H: Handlers='):
                    continue
                handlers = line.split('=', 1)[1].split()
                if 'kbd' not in handlers:
                    continue
                for handler in handlers:
                    if handler.startswith('event'):
                        fallback.append(f'/dev/input/{handler}')
    except OSError:
        return []
    return sorted(set(fallback))


class EvdevKeyboardThread(threading.Thread):
    def __init__(self, core, stop_event, device_paths, grab_device, poll_timeout):
        super().__init__(daemon=True)
        self.core = core
        self.stop_event = stop_event
        self.device_paths = device_paths
        self.grab_device = bool(grab_device)
        self.poll_timeout = max(0.001, float(poll_timeout))

    def run(self):
        event_format = 'llHHI'
        event_size = struct.calcsize(event_format)
        fd_to_path = {}
        buffers = {}
        ctrl_pressed = False

        try:
            for path in self.device_paths:
                fd = None
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    if self.grab_device:
                        fcntl.ioctl(fd, EVIOCGRAB, 1)
                except OSError:
                    try:
                        if fd is not None:
                            os.close(fd)
                    except Exception:
                        pass
                    continue
                fd_to_path[fd] = path
                buffers[fd] = b''

            if not fd_to_path:
                self.stop_event.set()
                return

            while rclpy.ok() and not self.stop_event.is_set():
                ready_fds, _, _ = select.select(list(fd_to_path.keys()), [], [], self.poll_timeout)
                if not ready_fds:
                    continue

                for fd in ready_fds:
                    try:
                        chunk = os.read(fd, event_size * 128)
                    except BlockingIOError:
                        continue
                    except OSError:
                        continue

                    if not chunk:
                        continue

                    buffer = buffers[fd] + chunk
                    event_count = len(buffer) // event_size
                    offset = 0

                    for _ in range(event_count):
                        _, _, event_type, code, value = struct.unpack_from(event_format, buffer, offset)
                        offset += event_size

                        if event_type != EV_KEY:
                            continue

                        if code in (KEY_LEFTCTRL, KEY_RIGHTCTRL):
                            ctrl_pressed = value in (1, 2)
                            continue

                        if code == KEY_C and value == 1 and ctrl_pressed:
                            self.stop_event.set()
                            break

                        key = KEY_CODE_TO_CHAR.get(code)
                        if key is None:
                            continue

                        timestamp = time.monotonic()
                        if key in MOVE_BINDINGS:
                            if value == 0:
                                self.core.handle_key_release(key, timestamp)
                            elif value in (1, 2):
                                self.core.handle_key_press(key, timestamp)
                        elif key in SPEED_BINDINGS and value == 1:
                            self.core.handle_key_press(key, timestamp)

                    buffers[fd] = buffer[offset:]
                    if self.stop_event.is_set():
                        break
        finally:
            self.core.clear_move_keys(time.monotonic())
            for fd in fd_to_path:
                try:
                    if self.grab_device:
                        fcntl.ioctl(fd, EVIOCGRAB, 0)
                except OSError:
                    pass
                try:
                    os.close(fd)
                except OSError:
                    pass


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop_node')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    params = declare_common_parameters(node)
    evdev_device = node.declare_parameter('evdev_device', '', read_only_descriptor).value
    grab_device = node.declare_parameter('grab_device', False, read_only_descriptor).value
    evdev_poll_timeout = node.declare_parameter('evdev_poll_timeout', 0.005, read_only_descriptor).value

    core = create_core_from_params(params)
    publisher = create_twist_publisher_adapter(node, params)
    publish_period = 1.0 / max(1.0, float(params['publish_rate']))
    node.create_timer(publish_period, lambda: publisher.publish(core.snapshot(time.monotonic())))

    device_paths = discover_evdev_keyboard_devices(str(evdev_device))
    readable_devices = [path for path in device_paths if os.access(path, os.R_OK)]
    if not readable_devices:
        node.get_logger().error('No readable keyboard evdev device found. Please set evdev_device.')
        node.destroy_node()
        rclpy.shutdown()
        return

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    stop_event = threading.Event()
    keyboard_thread = EvdevKeyboardThread(
        core=core,
        stop_event=stop_event,
        device_paths=readable_devices,
        grab_device=grab_device,
        poll_timeout=evdev_poll_timeout,
    )

    try:
        node.get_logger().warn('Using legacy evdev keyboard backend. This mode may conflict with desktop input and is not recommended for daily use.')
        print(COMMON_KEYBOARD_MESSAGE)
        keyboard_thread.start()

        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)
        core.emergency_stop(time.monotonic())
        publisher.publish_zero(core)
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == '__main__':
    main()
