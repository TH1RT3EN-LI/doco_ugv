import math
import threading
import time
from dataclasses import dataclass


MOVE_BINDINGS = {
    "w": (1.0, 0.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0, 0.0),
    "d": (0.0, -1.0, 0.0, 0.0),
    "q": (0.0, 0.0, 0.0, 1.0),
    "e": (0.0, 0.0, 0.0, -1.0),
    "x": (0.0, 0.0, 0.0, 0.0),
}

SPEED_BINDINGS = {
    "i": (1.0, 0.0),
    "k": (-1.0, 0.0),
    "o": (0.0, 1.0),
    "l": (0.0, -1.0),
}


@dataclass(frozen=True)
class TeleopCommand:
    linear_x: float
    linear_y: float
    linear_z: float
    angular_z: float
    linear_speed: float
    angular_speed: float
    active_keys: tuple[str, ...]


class KeyboardTeleopCore:
    def __init__(
        self,
        *,
        linear_speed: float,
        angular_speed: float,
        speed_step: float,
        turn_step: float,
        accel_limit_linear: float,
        decel_limit_linear: float,
        accel_limit_angular: float,
        decel_limit_angular: float,
        idle_timeout_sec: float,
    ) -> None:
        self._lock = threading.Lock()
        self._pressed_move_keys: set[str] = set()
        self._move_key_last_seen: dict[str, float] = {}
        self._linear_speed = max(0.0, float(linear_speed))
        self._angular_speed = max(0.0, float(angular_speed))
        self._speed_step = max(0.0, float(speed_step))
        self._turn_step = max(0.0, float(turn_step))
        self._accel_limit_linear = max(1e-6, float(accel_limit_linear))
        self._decel_limit_linear = max(1e-6, float(decel_limit_linear))
        self._accel_limit_angular = max(1e-6, float(accel_limit_angular))
        self._decel_limit_angular = max(1e-6, float(decel_limit_angular))
        self._idle_timeout_sec = max(0.0, float(idle_timeout_sec))

        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_linear_z = 0.0
        self._current_angular_z = 0.0
        self._last_input_time = time.monotonic()
        self._last_update_time = None

    def handle_key_press(self, key: str, now: float | None = None) -> bool:
        key = self._normalize_key(key)
        if key is None:
            return False

        timestamp = self._coerce_now(now)
        with self._lock:
            self._last_input_time = timestamp
            if key == "x":
                self._clear_move_keys_locked()
                self._zero_command_locked()
                self._last_update_time = timestamp
                return True

            if key in MOVE_BINDINGS:
                self._pressed_move_keys.add(key)
                self._move_key_last_seen[key] = timestamp
                return True

            if key in SPEED_BINDINGS:
                linear_step_direction, angular_step_direction = SPEED_BINDINGS[key]
                self._linear_speed = max(0.0, self._linear_speed + linear_step_direction * self._speed_step)
                self._angular_speed = max(0.0, self._angular_speed + angular_step_direction * self._turn_step)
                return True

        return False

    def handle_key_release(self, key: str, now: float | None = None) -> bool:
        key = self._normalize_key(key)
        if key is None or key not in MOVE_BINDINGS:
            return False

        timestamp = self._coerce_now(now)
        with self._lock:
            self._last_input_time = timestamp
            if key != "x":
                self._pressed_move_keys.discard(key)
                self._move_key_last_seen.pop(key, None)
        return True

    def clear_move_keys(self, now: float | None = None) -> None:
        timestamp = self._coerce_now(now)
        with self._lock:
            self._clear_move_keys_locked()
            self._last_input_time = timestamp

    def emergency_stop(self, now: float | None = None) -> None:
        timestamp = self._coerce_now(now)
        with self._lock:
            self._clear_move_keys_locked()
            self._zero_command_locked()
            self._last_input_time = timestamp
            self._last_update_time = timestamp

    def expire_stale_move_keys(self, stale_after_sec: float, now: float | None = None) -> None:
        timeout = max(0.0, float(stale_after_sec))
        timestamp = self._coerce_now(now)
        with self._lock:
            stale_keys = [
                key for key, seen_at in self._move_key_last_seen.items()
                if (timestamp - seen_at) > timeout
            ]
            for key in stale_keys:
                self._pressed_move_keys.discard(key)
                self._move_key_last_seen.pop(key, None)

    def snapshot(self, now: float | None = None, stale_key_timeout: float | None = None) -> TeleopCommand:
        timestamp = self._coerce_now(now)
        with self._lock:
            if stale_key_timeout is not None:
                self._expire_stale_locked(timestamp, max(0.0, float(stale_key_timeout)))

            if self._idle_timeout_sec > 0.0 and (timestamp - self._last_input_time) > self._idle_timeout_sec:
                self._clear_move_keys_locked()

            target_x, target_y, target_z, target_th = self._target_vector_locked()
            dt = 0.0 if self._last_update_time is None else max(0.0, timestamp - self._last_update_time)

            target_linear_x = target_x * self._linear_speed
            target_linear_y = target_y * self._linear_speed
            target_linear_z = target_z * self._linear_speed
            target_angular_z = target_th * self._angular_speed

            self._current_linear_x = self._slew_axis(
                self._current_linear_x,
                target_linear_x,
                dt,
                self._accel_limit_linear,
                self._decel_limit_linear,
            )
            self._current_linear_y = self._slew_axis(
                self._current_linear_y,
                target_linear_y,
                dt,
                self._accel_limit_linear,
                self._decel_limit_linear,
            )
            self._current_linear_z = self._slew_axis(
                self._current_linear_z,
                target_linear_z,
                dt,
                self._accel_limit_linear,
                self._decel_limit_linear,
            )
            self._current_angular_z = self._slew_axis(
                self._current_angular_z,
                target_angular_z,
                dt,
                self._accel_limit_angular,
                self._decel_limit_angular,
            )
            self._last_update_time = timestamp

            return TeleopCommand(
                linear_x=self._current_linear_x,
                linear_y=self._current_linear_y,
                linear_z=self._current_linear_z,
                angular_z=self._current_angular_z,
                linear_speed=self._linear_speed,
                angular_speed=self._angular_speed,
                active_keys=tuple(sorted(self._pressed_move_keys)),
            )

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "linear_speed": self._linear_speed,
                "angular_speed": self._angular_speed,
                "active_keys": tuple(sorted(self._pressed_move_keys)),
            }

    def zero_command(self) -> TeleopCommand:
        with self._lock:
            self._zero_command_locked()
            active_keys = tuple(sorted(self._pressed_move_keys))
            return TeleopCommand(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=0.0,
                linear_speed=self._linear_speed,
                angular_speed=self._angular_speed,
                active_keys=active_keys,
            )

    def _target_vector_locked(self) -> tuple[float, float, float, float]:
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        for key in self._pressed_move_keys:
            dx, dy, dz, dth = MOVE_BINDINGS[key]
            x += dx
            y += dy
            z += dz
            th += dth

        return (
            max(-1.0, min(1.0, x)),
            max(-1.0, min(1.0, y)),
            max(-1.0, min(1.0, z)),
            max(-1.0, min(1.0, th)),
        )

    def _clear_move_keys_locked(self) -> None:
        self._pressed_move_keys.clear()
        self._move_key_last_seen.clear()

    def _zero_command_locked(self) -> None:
        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_linear_z = 0.0
        self._current_angular_z = 0.0

    def _expire_stale_locked(self, now: float, timeout: float) -> None:
        stale_keys = [
            key for key, seen_at in self._move_key_last_seen.items()
            if (now - seen_at) > timeout
        ]
        for key in stale_keys:
            self._pressed_move_keys.discard(key)
            self._move_key_last_seen.pop(key, None)

    @staticmethod
    def _normalize_key(key: str | None) -> str | None:
        if not key:
            return None
        normalized = key.lower()
        if normalized in MOVE_BINDINGS or normalized in SPEED_BINDINGS:
            return normalized
        return None

    @staticmethod
    def _coerce_now(now: float | None) -> float:
        return time.monotonic() if now is None else float(now)

    @staticmethod
    def _slew_axis(current: float, target: float, dt: float, accel_limit: float, decel_limit: float) -> float:
        if dt <= 0.0:
            return target
        if math.isclose(current, target, abs_tol=1e-9):
            return target

        if current == 0.0:
            limit = accel_limit if abs(target) > 0.0 else decel_limit
        elif current * target < 0.0 or abs(target) < abs(current):
            limit = decel_limit
        else:
            limit = accel_limit

        max_delta = limit * dt
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)
