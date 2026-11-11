"""FSC-2A motion controller monitor and command GUI."""

from __future__ import annotations

import ctypes
import math
import queue
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from tkinter import messagebox, ttk
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - friendly startup failure
    raise SystemExit("缺少 pyserial，请执行：python -m pip install -r requirements.txt") from exc

from fsc2a_protocol import (
    AXIS_LEAD,
    AXIS_SUBDIVISION,
    COIL_ABSOLUTE,
    COIL_RELATIVE_FORWARD,
    COIL_RELATIVE_REVERSE,
    COIL_SET_POSITION_ZERO,
    COIL_STOP,
    MOVE_DISTANCE,
    SPEED_FORWARD,
    SPEED_REVERSE,
    TRANSMITTER_GROSS_FORCE,
    TRANSMITTER_MANUAL_ZERO,
    TRANSMITTER_STATUS,
    ModbusRTUClient,
    MonitorValues,
    int32_from_registers,
    uint32_from_registers,
)


APP_TITLE = "FSC-2A 单轴运动控制器"
BG = "#10091D"
PANEL = "#211438"
FIELD = "#170D29"
BORDER = "#4C3575"
TEXT = "#F4F0FF"
MUTED = "#B6A8D1"
ACCENT = "#A78BFA"
GREEN = "#5EE0B1"
RED = "#FF6B9E"
AMBER = "#F5C66D"

TRAVEL_MM = 100
HALF_TRAVEL_MM = 50
# FSC "reverse" now points toward the physical positive/up limit.  Negate
# controller counts so the user coordinate still increases upward.
POSITION_DIRECTION = -1.0
CALIBRATION_TIMEOUT_SECONDS = 120.0
FORCE_POLL_INTERVAL_SECONDS = 0.15
STOP_RETRY_COUNT = 3
STOP_RETRY_INTERVAL_SECONDS = 0.05


def _enable_high_dpi() -> None:
    """Prevent Windows display scaling from bitmap-stretching the Tk window."""
    if sys.platform != "win32":
        return
    try:
        # Per-monitor DPI awareness v2 (Windows 10 1703+).
        ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
    except (AttributeError, OSError):
        try:
            # Fallback for older Windows versions.
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            try:
                ctypes.windll.user32.SetProcessDPIAware()
            except (AttributeError, OSError):
                pass


_enable_high_dpi()


@dataclass(frozen=True)
class ConnectionSettings:
    port: str
    baudrate: int
    slave_id: int
    poll_interval: float
    driver_subdivision: int = 2500


class ControllerWorker(threading.Thread):
    """Own the serial port so polling and commands can never collide."""

    def __init__(self, settings: ConnectionSettings, events: queue.Queue[tuple[str, Any]]):
        super().__init__(daemon=True, name="fsc2a-worker")
        self.settings = settings
        self.events = events
        self.commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.shutdown_event = threading.Event()
        self.automation: dict[str, Any] | None = None
        self.latest_values: MonitorValues | None = None
        self.lead_mm: int | None = None
        self.subdivision: int | None = None
        self.pulses_per_mm: float | None = None
        self.force_limit: float | None = None
        self.force_decimal_places: int | None = None
        self.force_transmitter_slave: int | None = None
        self.force_limit_tripped = False
        self.force_poll_errors = 0
        self.emergency_stop_event = threading.Event()
        self.emergency_stop_lock = threading.Lock()
        self.emergency_stop_reason = "用户点击急停"

    def submit_coil(self, name: str, address: int) -> None:
        self.commands.put(("coil", (name, address)))

    def submit_emergency_stop(self, reason: str = "用户点击急停") -> None:
        """Request a stop ahead of queued commands and periodic polling."""
        with self.emergency_stop_lock:
            self.emergency_stop_reason = reason
        self.emergency_stop_event.set()

    def _take_emergency_stop_reason(self) -> str | None:
        if not self.emergency_stop_event.is_set():
            return None
        with self.emergency_stop_lock:
            reason = self.emergency_stop_reason
            self.emergency_stop_event.clear()
        return reason

    def submit_calibration(self) -> None:
        self.commands.put(("calibrate", None))

    def submit_home(self) -> None:
        self.commands.put(("home", None))

    def submit_read_speeds(self) -> None:
        self.commands.put(("read_speeds", None))

    def submit_set_speeds(self, forward: int, reverse: int) -> None:
        self.commands.put(("set_speeds", (forward, reverse)))

    def submit_transmitter_zero(self, slave_id: int) -> None:
        self.commands.put(("transmitter_zero", slave_id))

    def submit_force_limit(self, slave_id: int, maximum_force: float) -> None:
        self.commands.put(("force_limit", (slave_id, maximum_force)))

    def submit_disable_force_limit(self) -> None:
        self.commands.put(("disable_force_limit", None))

    def stop(self) -> None:
        self.shutdown_event.set()

    def _emit(self, kind: str, payload: Any = None) -> None:
        self.events.put((kind, payload))

    def _load_and_sync_axis_config(self, client: ModbusRTUClient) -> None:
        """Make the FSC pulse/revolution value match the connected ISS35."""
        axis_registers = client.read_holding_registers(AXIS_LEAD, 4)
        lead_mm = uint32_from_registers(axis_registers[0], axis_registers[1])
        previous_subdivision = uint32_from_registers(
            axis_registers[2], axis_registers[3]
        )
        requested_subdivision = self.settings.driver_subdivision

        if previous_subdivision != requested_subdivision:
            client.write_int32(AXIS_SUBDIVISION, requested_subdivision)
            verify_registers = client.read_holding_registers(AXIS_SUBDIVISION, 2)
            verified_subdivision = uint32_from_registers(*verify_registers)
            if verified_subdivision != requested_subdivision:
                raise RuntimeError(
                    "FSC-2A 细分写入校验失败："
                    f"期望 {requested_subdivision}，回读 {verified_subdivision}"
                )
            self._emit(
                "axis_config_corrected",
                (previous_subdivision, verified_subdivision),
            )
            subdivision = verified_subdivision
        else:
            subdivision = previous_subdivision

        if lead_mm <= 0 or subdivision <= 0:
            raise RuntimeError(
                f"导程或细分参数无效：导程={lead_mm}，细分={subdivision}"
            )
        self.lead_mm = lead_mm
        self.subdivision = subdivision
        self.pulses_per_mm = subdivision / lead_mm
        self._emit(
            "axis_config",
            (lead_mm, subdivision, self.pulses_per_mm),
        )

    def _send_stop_with_retries(self, client: ModbusRTUClient) -> str | None:
        """Send the controller stop coil, retrying if its echo is lost."""
        last_error: Exception | None = None
        for attempt in range(STOP_RETRY_COUNT):
            try:
                client.write_single_coil(COIL_STOP, True)
                return None
            except Exception as exc:
                last_error = exc
                if attempt + 1 < STOP_RETRY_COUNT:
                    self.shutdown_event.wait(STOP_RETRY_INTERVAL_SECONDS)
        return str(last_error) if last_error is not None else "未知停止错误"

    def _emergency_stop(
        self,
        client: ModbusRTUClient,
        *,
        latch_force_protection: bool = False,
    ) -> tuple[str | None, str | None]:
        """Latch the software state first, then issue the same stop for every source."""
        interrupted_kind = self.automation["kind"] if self.automation is not None else None
        self.automation = None
        if latch_force_protection:
            self.force_limit_tripped = True
        stop_error = self._send_stop_with_retries(client)
        return interrupted_kind, stop_error

    def _poll_transmitter_force(self, client: ModbusRTUClient) -> bool:
        if (
            self.force_limit is None
            or self.force_decimal_places is None
            or self.force_transmitter_slave is None
        ):
            return False
        registers = client.read_holding_registers(
            TRANSMITTER_GROSS_FORCE,
            2,
            slave_id=self.force_transmitter_slave,
        )
        raw_force = int32_from_registers(*registers)
        force = raw_force / (10 ** self.force_decimal_places)
        self.force_poll_errors = 0
        self._emit("force_value", (force, self.force_decimal_places))
        if self.force_limit_tripped or abs(force) <= self.force_limit:
            return False

        interrupted_kind, stop_error = self._emergency_stop(
            client,
            latch_force_protection=True,
        )
        self._emit(
            "force_limit_tripped",
            (force, self.force_limit, interrupted_kind, stop_error),
        )
        return True

    def _start_calibration(
        self,
        client: ModbusRTUClient,
    ) -> None:
        # 0x004A/0x004B are read-only.  Coil 0x000D is the controller's
        # supported way to clear its internal current-position value.
        client.write_single_coil(COIL_SET_POSITION_ZERO, True)
        self._emit("position_reference", (0.0, 0.0))
        client.write_int32(MOVE_DISTANCE, TRAVEL_MM)
        client.write_single_coil(COIL_RELATIVE_REVERSE, True)
        self.automation = {
            "kind": "calibration",
            "phase": "seek_limit",
            "started_at": time.monotonic(),
            "start_position": 0,
            "saw_motion": False,
            "saw_speed": False,
            "zero_polls": 0,
        }
        self._emit("operation_step", "标定原点：当前位置已清零，正在反转（向上）寻找正限位…")

    def _start_home(self, client: ModbusRTUClient) -> None:
        client.write_int32(MOVE_DISTANCE, 0)
        client.write_single_coil(COIL_ABSOLUTE, True)
        start_position = self.latest_values.position if self.latest_values is not None else None
        self.automation = {
            "kind": "home",
            "phase": "return_zero",
            "started_at": time.monotonic(),
            "start_position": start_position,
            "saw_motion": False,
            "saw_speed": False,
            "zero_polls": 0,
        }
        self._emit("operation_step", "启动回零：正在返回已标定的 0 mm 原点…")

    def _cancel_automation(self, reason: str) -> None:
        if self.automation is not None:
            self.automation = None
            self._emit("operation_cancelled", reason)

    def _fail_automation(self, client: ModbusRTUClient, message: str) -> None:
        self.automation = None
        stop_error = self._send_stop_with_retries(client)
        if stop_error is not None:
            message = f"{message}；停止命令重试 {STOP_RETRY_COUNT} 次仍失败：{stop_error}"
        self._emit("operation_error", message)

    def _advance_automation(self, client: ModbusRTUClient, values: MonitorValues) -> None:
        state = self.automation
        if state is None:
            return
        now = time.monotonic()
        if now - state["started_at"] > CALIBRATION_TIMEOUT_SECONDS:
            self._fail_automation(client, "动作超时，已发送停止命令")
            return

        if state["kind"] == "home":
            start = state["start_position"]
            moved = start is not None and values.position != start
            if values.speed != 0 or moved:
                state["saw_motion"] = True
            state["zero_polls"] = state["zero_polls"] + 1 if values.speed == 0 else 0
            at_zero = values.position == 0
            no_move_needed = start == 0 and now - state["started_at"] >= 0.4
            if (at_zero and state["zero_polls"] >= 2 and now - state["started_at"] >= 0.2) or no_move_needed:
                self.automation = None
                self._emit("home_done", values.position)
            return

        phase = state["phase"]
        if phase == "seek_limit":
            start = state["start_position"]
            moved_distance = 0 if start is None else abs(values.position - start)
            if values.speed != 0 or moved_distance > 0:
                state["saw_motion"] = True
            if values.speed != 0:
                state["saw_speed"] = True

            state["zero_polls"] = state["zero_polls"] + 1 if values.speed == 0 else 0
            if state["saw_motion"] and state["zero_polls"] >= 3:
                stop_error = self._send_stop_with_retries(client)
                if stop_error is not None:
                    self.automation = None
                    self._emit(
                        "operation_error",
                        f"控制器已在正限位停止，但停止命令重试 {STOP_RETRY_COUNT} 次仍未确认：{stop_error}",
                    )
                    return
                # The FSC-2A can only set its internal position to zero, not
                # to an arbitrary +50 mm. Zero it at the positive limit and
                # expose the desired -50..+50 coordinate through a UI offset.
                client.write_single_coil(COIL_SET_POSITION_ZERO, True)
                self._emit("position_reference", (50.0, 50.0))
                state.update(phase="wait_limit_stop", zero_polls=0)
                self._emit(
                    "operation_step",
                    "控制器已在物理正限位停止，当前位置标记为 +50.00 mm；等待电机停稳…",
                )
            return

        if phase == "wait_limit_stop":
            state["zero_polls"] = state["zero_polls"] + 1 if values.speed == 0 else 0
            if state["zero_polls"] >= 2:
                client.write_int32(MOVE_DISTANCE, HALF_TRAVEL_MM)
                client.write_single_coil(COIL_RELATIVE_FORWARD, True)
                state.update(
                    phase="backoff",
                    started_at=now,
                    start_position=values.position,
                    saw_motion=False,
                    saw_speed=False,
                    zero_polls=0,
                )
                self._emit("operation_step", "限位端已停稳，开始正转（向下）移动 50 mm…")
            return

        if phase == "backoff":
            if not self.pulses_per_mm:
                self._fail_automation(client, "未读取到有效的导程/细分参数，无法换算当前位置")
                return
            start = state["start_position"]
            moved_distance = abs(values.position - start)
            if values.speed != 0 or moved_distance > 0:
                state["saw_motion"] = True
            if values.speed != 0:
                state["saw_speed"] = True
            state["zero_polls"] = state["zero_polls"] + 1 if values.speed == 0 else 0

            logical_position_mm = (
                POSITION_DIRECTION * values.position / self.pulses_per_mm
                + HALF_TRAVEL_MM
            )
            if state["saw_motion"] and logical_position_mm <= 0:
                stop_error = self._send_stop_with_retries(client)
                if stop_error is not None:
                    self.automation = None
                    self._emit(
                        "operation_error",
                        f"到达逻辑原点，但停止命令重试 {STOP_RETRY_COUNT} 次仍未确认：{stop_error}",
                    )
                    return
                state.update(
                    phase="wait_center_stop",
                    started_at=now,
                    zero_polls=0,
                )
                self._emit(
                    "operation_step",
                    f"已到达逻辑原点 {logical_position_mm:.2f} mm，已发送停止命令；等待电机停稳…",
                )
            elif state["saw_motion"] and state["zero_polls"] >= 2:
                self._fail_automation(
                    client,
                    f"向下运动在 {logical_position_mm:.2f} mm 提前停止，尚未到达 0 mm",
                )
            return

        if phase == "wait_center_stop":
            state["zero_polls"] = state["zero_polls"] + 1 if values.speed == 0 else 0
            if state["zero_polls"] >= 2:
                client.write_single_coil(COIL_SET_POSITION_ZERO, True)
                self._emit("position_reference", (0.0, 0.0))
                self.automation = None
                self._emit("calibration_done", HALF_TRAVEL_MM)

    def run(self) -> None:
        client: ModbusRTUClient | None = None
        try:
            port = serial.Serial(
                port=self.settings.port,
                baudrate=self.settings.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.35,
                write_timeout=0.35,
            )
            client = ModbusRTUClient(port, self.settings.slave_id)
            self._load_and_sync_axis_config(client)
            self._emit("connected", self.settings)
            next_poll = 0.0
            next_force_poll = 0.0
            consecutive_errors = 0

            while not self.shutdown_event.is_set():
                emergency_reason = self._take_emergency_stop_reason()
                if emergency_reason is not None:
                    interrupted_kind, stop_error = self._emergency_stop(client)
                    self._emit(
                        "emergency_stop_result",
                        (emergency_reason, interrupted_kind, stop_error),
                    )
                    continue

                try:
                    command_kind, command_payload = self.commands.get_nowait()
                except queue.Empty:
                    pass
                else:
                    try:
                        if command_kind == "coil":
                            command_name, address = command_payload
                            if address == COIL_STOP:
                                interrupted_kind, stop_error = self._emergency_stop(client)
                                self._emit(
                                    "emergency_stop_result",
                                    ("用户点击急停", interrupted_kind, stop_error),
                                )
                            else:
                                client.write_single_coil(address, True)
                                self._emit("command_ok", command_name)
                        elif command_kind == "calibrate":
                            if self.force_limit_tripped:
                                raise RuntimeError("超力保护已锁定，请排除原因并重新布防")
                            if self.automation is not None:
                                raise RuntimeError("当前已有运动流程正在执行")
                            self._start_calibration(client)
                        elif command_kind == "home":
                            if self.force_limit_tripped:
                                raise RuntimeError("超力保护已锁定，请排除原因并重新布防")
                            if self.automation is not None:
                                raise RuntimeError("当前已有运动流程正在执行")
                            self._start_home(client)
                        elif command_kind == "read_speeds":
                            forward_registers = client.read_holding_registers(SPEED_FORWARD, 2)
                            reverse_registers = client.read_holding_registers(SPEED_REVERSE, 2)
                            self._emit(
                                "speed_values",
                                (
                                    uint32_from_registers(*forward_registers),
                                    uint32_from_registers(*reverse_registers),
                                ),
                            )
                        elif command_kind == "set_speeds":
                            if self.automation is not None:
                                raise RuntimeError("运动过程中不能修改速度")
                            forward, reverse = command_payload
                            client.write_int32(SPEED_FORWARD, forward)
                            client.write_int32(SPEED_REVERSE, reverse)
                            self._emit("speed_set_ok", (forward, reverse))
                        elif command_kind == "transmitter_zero":
                            client.write_multiple_registers(
                                TRANSMITTER_MANUAL_ZERO,
                                [0x0001],
                                slave_id=command_payload,
                            )
                            self._emit("command_ok", "变送器清零")
                        elif command_kind == "force_limit":
                            force_slave, maximum_force = command_payload
                            status = client.read_holding_registers(
                                TRANSMITTER_STATUS,
                                1,
                                slave_id=force_slave,
                            )[0]
                            decimal_places = status & 0x0007
                            if decimal_places > 6:
                                raise RuntimeError(f"变送器返回无效小数位：{decimal_places}")
                            self.force_limit = maximum_force
                            self.force_decimal_places = decimal_places
                            self.force_transmitter_slave = force_slave
                            self.force_limit_tripped = False
                            self.force_poll_errors = 0
                            next_force_poll = 0.0
                            self._emit(
                                "force_limit_configured",
                                (maximum_force, decimal_places, force_slave),
                            )
                            if self._poll_transmitter_force(client):
                                continue
                        elif command_kind == "disable_force_limit":
                            self.force_limit = None
                            self.force_decimal_places = None
                            self.force_transmitter_slave = None
                            self.force_limit_tripped = False
                            self.force_poll_errors = 0
                            self._emit("force_limit_disabled")
                        consecutive_errors = 0
                    except Exception as exc:
                        if command_kind == "coil":
                            command_name = command_payload[0]
                        elif command_kind in ("read_speeds", "set_speeds"):
                            command_name = "读取速度" if command_kind == "read_speeds" else "设置速度"
                        elif command_kind == "transmitter_zero":
                            command_name = "变送器清零"
                        elif command_kind == "force_limit":
                            command_name = "设置最大力保护"
                        elif command_kind == "disable_force_limit":
                            command_name = "停用最大力保护"
                        else:
                            command_name = "标定原点" if command_kind == "calibrate" else "启动回零"
                        if command_kind in ("calibrate", "home"):
                            self.automation = None
                        self._emit("command_error", (command_name, str(exc)))

                # A button press that arrived during the preceding transaction
                # must pre-empt both periodic polls on the next iteration.
                if self.emergency_stop_event.is_set():
                    continue

                now = time.monotonic()
                safety_stop_this_cycle = False
                if self.force_limit is not None and now >= next_force_poll:
                    try:
                        safety_stop_this_cycle = self._poll_transmitter_force(client)
                    except Exception as exc:
                        self.force_poll_errors += 1
                        self._emit("force_poll_error", (str(exc), self.force_poll_errors))
                        if (
                            self.automation is not None
                            and self.force_poll_errors >= 3
                            and not self.force_limit_tripped
                        ):
                            interrupted_kind, stop_error = self._emergency_stop(
                                client,
                                latch_force_protection=True,
                            )
                            self._emit(
                                "force_monitor_failed",
                                (str(exc), interrupted_kind, stop_error),
                            )
                            safety_stop_this_cycle = True
                    next_force_poll = time.monotonic() + FORCE_POLL_INTERVAL_SECONDS

                if safety_stop_this_cycle or self.emergency_stop_event.is_set():
                    continue

                now = time.monotonic()
                if now >= next_poll:
                    try:
                        values = client.read_monitor_values()
                        self.latest_values = values
                        self._emit("values", values)
                        self._advance_automation(client, values)
                        consecutive_errors = 0
                    except Exception as exc:
                        consecutive_errors += 1
                        self._emit("poll_error", (str(exc), consecutive_errors))
                    interval = min(self.settings.poll_interval, 0.05) if self.automation else self.settings.poll_interval
                    next_poll = time.monotonic() + interval
                self.shutdown_event.wait(0.01)
        except Exception as exc:
            self._emit("connect_error", str(exc))
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            self._emit("disconnected", self)


class FSC2AApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1100x760")
        self.minsize(1000, 700)
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.worker: ControllerWorker | None = None
        self.connected = False
        self.calibrated = False
        self.operation_busy = False
        self.transmitter_zero_pending = False
        self.force_limit_pending = False
        self.force_limit_active = False
        self.force_limit_tripped = False
        self.last_force_poll_error = ""
        self.speed_window: tk.Toplevel | None = None
        self.axis_lead_mm: int | None = None
        self.axis_subdivision: int | None = None
        self.pulses_per_mm: float | None = None
        self.position_offset_mm = 0.0
        self.previous_inputs: int | None = None
        self.last_poll_error = ""

        self._configure_styles()
        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_events)

    def _configure_styles(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=TEXT, font=("Microsoft YaHei UI", 10))
        style.configure("Panel.TLabel", background=PANEL, foreground=TEXT, font=("Microsoft YaHei UI", 10))
        style.configure("Muted.Panel.TLabel", background=PANEL, foreground=MUTED, font=("Microsoft YaHei UI", 9))
        style.configure("Title.TLabel", background=BG, foreground=TEXT, font=("Microsoft YaHei UI", 18, "bold"))
        style.configure("Value.Panel.TLabel", background=PANEL, foreground=ACCENT, font=("Consolas", 27, "bold"))
        style.configure("TButton", foreground=TEXT, background="#352354", bordercolor=BORDER,
                        font=("Microsoft YaHei UI", 10), padding=(12, 7))
        style.map("TButton", background=[("active", "#4B3370"), ("disabled", "#2A2038")],
                  foreground=[("disabled", "#746987")])
        style.configure("TCombobox", fieldbackground=FIELD, background=PANEL, foreground=TEXT,
                        arrowcolor=TEXT, bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER,
                        selectbackground=ACCENT, selectforeground="#160D29")
        style.map("TCombobox", fieldbackground=[("readonly", FIELD)],
                  foreground=[("readonly", TEXT)], selectbackground=[("readonly", FIELD)])
        style.configure("TSpinbox", fieldbackground=FIELD, background=PANEL, foreground=TEXT,
                        arrowcolor=TEXT, bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
        style.configure("TEntry", fieldbackground=FIELD, foreground=TEXT, insertcolor=TEXT,
                        bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
        style.configure("TSeparator", background=BORDER)
        style.configure("Stop.TButton", foreground="white", background="#C42D68",
                        bordercolor="#E35A8D", font=("Microsoft YaHei UI", 12, "bold"))
        style.map("Stop.TButton", background=[("active", "#E0447F"), ("disabled", "#2A2038")])
        style.configure("Home.TButton", foreground="#150C25", background=ACCENT,
                        bordercolor="#C4B5FD", font=("Microsoft YaHei UI", 12, "bold"))
        style.map("Home.TButton", background=[("active", "#C4B5FD"), ("disabled", "#2A2038")],
                  foreground=[("disabled", "#746987")])

    def _build_ui(self) -> None:
        outer = ttk.Frame(self, padding=18)
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer)
        header.pack(fill="x", pady=(0, 14))
        ttk.Label(header, text=APP_TITLE, style="Title.TLabel").pack(side="left")
        self.status_dot = tk.Label(header, text="●", bg=BG, fg=RED, font=("Segoe UI Symbol", 15))
        self.status_dot.pack(side="right", padx=(6, 0))
        self.status_text = ttk.Label(header, text="未连接")
        self.status_text.pack(side="right")

        connection = ttk.Frame(outer, style="Panel.TFrame", padding=14)
        connection.pack(fill="x", pady=(0, 12))
        fields = [
            ("串口", 0), ("波特率", 2), ("站号", 4), ("轮询间隔", 6)
        ]
        for label, column in fields:
            ttk.Label(connection, text=label, style="Panel.TLabel").grid(row=0, column=column, padx=(0, 6), sticky="w")

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(0, 14))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(connection, textvariable=self.baud_var, width=9, state="readonly",
                                       values=("2400", "4800", "9600", "19200", "38400", "57600", "115200"))
        self.baud_combo.grid(row=0, column=3, padx=(0, 14))
        self.slave_var = tk.StringVar(value="1")
        ttk.Spinbox(connection, from_=1, to=255, textvariable=self.slave_var, width=6).grid(row=0, column=5, padx=(0, 14))
        self.poll_var = tk.StringVar(value="200 ms")
        self.poll_combo = ttk.Combobox(connection, textvariable=self.poll_var, width=9, state="readonly",
                                       values=("100 ms", "200 ms", "500 ms", "1000 ms"))
        self.poll_combo.grid(row=0, column=7, padx=(0, 14))
        ttk.Button(connection, text="刷新", command=self._refresh_ports).grid(row=0, column=8, padx=(0, 7))
        self.speed_button = ttk.Button(connection, text="速度设置", command=self._open_speed_window, state="disabled")
        self.speed_button.grid(row=0, column=9, padx=(0, 7))
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=10)
        connection.columnconfigure(11, weight=1)

        ttk.Label(
            connection,
            text="ISS35细分",
            style="Panel.TLabel",
        ).grid(row=1, column=0, padx=(0, 6), pady=(10, 0), sticky="w")
        self.driver_subdivision_var = tk.StringVar(value="2500")
        self.driver_subdivision_spinbox = ttk.Spinbox(
            connection,
            from_=200,
            to=60000,
            increment=1,
            textvariable=self.driver_subdivision_var,
            width=16,
        )
        self.driver_subdivision_spinbox.grid(
            row=1,
            column=1,
            padx=(0, 14),
            pady=(10, 0),
            sticky="w",
        )
        ttk.Label(
            connection,
            text="连接时同步到FSC-2A；必须与ISS35调试软件中的每转脉冲数一致",
            style="Muted.Panel.TLabel",
        ).grid(row=1, column=2, columnspan=9, pady=(10, 0), sticky="w")

        values = ttk.Frame(outer)
        values.pack(fill="x", pady=(0, 12))
        values.columnconfigure((0, 1, 2), weight=1, uniform="value")
        self.position_value = self._value_card(values, 0, "当前位置", "0", "mm · 0x004A / 0x004B")
        self.speed_value = self._value_card(values, 1, "当前速度", "0", "mm/s · 0x004C / 0x004D")
        self.input_value = self._value_card(values, 2, "输入口状态", "0x00000000", "32 位原始值 · 0x004E / 0x004F")

        action_panel = ttk.Frame(outer, style="Panel.TFrame", padding=14)
        action_panel.pack(fill="x", pady=(0, 12))
        ttk.Label(action_panel, text="运动命令", style="Panel.TLabel", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        ttk.Label(action_panel, text="命令使用 Modbus 0x05 写单线圈",
                  style="Muted.Panel.TLabel").pack(anchor="w", pady=(2, 12))
        action_buttons = ttk.Frame(action_panel, style="Panel.TFrame")
        action_buttons.pack(fill="x")
        action_buttons.columnconfigure((0, 1, 2), weight=1, uniform="action")
        self.stop_button = ttk.Button(action_buttons, text="■  停止运动  0x0004", style="Stop.TButton",
                                      command=self._send_stop, state="disabled")
        self.stop_button.grid(row=0, column=0, sticky="ew", padx=(0, 6), ipady=7)
        self.calibrate_button = ttk.Button(action_buttons, text="◎  标定原点", style="Home.TButton",
                                           command=self._send_calibration, state="disabled")
        self.calibrate_button.grid(row=0, column=1, sticky="ew", padx=6, ipady=7)
        self.home_button = ttk.Button(action_buttons, text="⌂  启动回零", style="Home.TButton",
                                      command=self._send_home, state="disabled")
        self.home_button.grid(row=0, column=2, sticky="ew", padx=(6, 0), ipady=7)

        transmitter_controls = ttk.Frame(action_panel, style="Panel.TFrame")
        transmitter_controls.pack(fill="x", pady=(10, 0))
        ttk.Label(transmitter_controls, text="变送器站号", style="Panel.TLabel").pack(side="left")
        self.transmitter_slave_var = tk.StringVar(value="2")
        self.transmitter_slave_spinbox = ttk.Spinbox(
            transmitter_controls,
            from_=1,
            to=247,
            textvariable=self.transmitter_slave_var,
            width=6,
        )
        self.transmitter_slave_spinbox.pack(side="left", padx=(8, 12))
        self.transmitter_zero_button = ttk.Button(
            transmitter_controls,
            text="变送器清零  0x005E",
            style="Home.TButton",
            command=self._send_transmitter_zero,
            state="disabled",
        )
        self.transmitter_zero_button.pack(side="left")
        ttk.Label(
            transmitter_controls,
            text="串口和波特率自动跟随 FUYU",
            style="Muted.Panel.TLabel",
        ).pack(side="left", padx=(12, 0))

        force_controls = ttk.Frame(action_panel, style="Panel.TFrame")
        force_controls.pack(fill="x", pady=(10, 0))
        self.force_value_label = ttk.Label(
            force_controls,
            text="当前力：--",
            style="Panel.TLabel",
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        self.force_value_label.pack(side="left", padx=(0, 18))
        ttk.Label(force_controls, text="最大力", style="Panel.TLabel").pack(side="left")
        self.maximum_force_var = tk.StringVar(value="100")
        self.maximum_force_entry = ttk.Entry(
            force_controls,
            textvariable=self.maximum_force_var,
            width=12,
        )
        self.maximum_force_entry.pack(side="left", padx=(8, 6))
        ttk.Label(
            force_controls,
            text="仪表显示单位",
            style="Muted.Panel.TLabel",
        ).pack(side="left", padx=(0, 12))
        self.force_limit_apply_button = ttk.Button(
            force_controls,
            text="应用并启用",
            command=self._apply_force_limit,
            state="disabled",
        )
        self.force_limit_apply_button.pack(side="left", padx=(0, 6))
        self.force_limit_disable_button = ttk.Button(
            force_controls,
            text="停用保护",
            command=self._disable_force_limit,
            state="disabled",
        )
        self.force_limit_disable_button.pack(side="left")

        self.force_limit_status = ttk.Label(
            action_panel,
            text="最大力保护未启用；比较正负力值的绝对值",
            style="Muted.Panel.TLabel",
        )
        self.force_limit_status.pack(anchor="w", pady=(8, 0))

        calibration_options = ttk.Frame(action_panel, style="Panel.TFrame")
        calibration_options.pack(fill="x", pady=(10, 0))
        ttk.Label(calibration_options,
                  text="标定流程：反转向上寻找正限位 → 控制器停止 → 正转向下 50 mm → 当前位置设为 0",
                  style="Muted.Panel.TLabel").pack(side="left")
        ttk.Label(action_panel, text="安全提示：盲走和回零会使机构运动，请确认正限位接线、方向和工作区域安全。",
                  style="Muted.Panel.TLabel").pack(anchor="w", pady=(8, 0))

        log_panel = ttk.Frame(outer, style="Panel.TFrame", padding=10)
        log_panel.pack(fill="both", expand=True)
        log_header = ttk.Frame(log_panel, style="Panel.TFrame")
        log_header.pack(fill="x")
        ttk.Label(log_header, text="通信记录", style="Panel.TLabel", font=("Microsoft YaHei UI", 10, "bold")).pack(side="left")
        ttk.Button(log_header, text="清空", command=lambda: self.log.delete("1.0", "end")).pack(side="right")
        self.log = tk.Text(log_panel, height=7, bg="#0C0715", fg="#DCD3EE", insertbackground=TEXT,
                           selectbackground="#60458A", selectforeground=TEXT, relief="flat",
                           padx=8, pady=6, font=("Consolas", 10), state="normal")
        self.log.pack(fill="both", expand=True, pady=(7, 0))
        self._log("程序已启动；请选择串口并连接。")

    def _value_card(self, parent: ttk.Frame, column: int, title: str, value: str, detail: str) -> ttk.Label:
        card = ttk.Frame(parent, style="Panel.TFrame", padding=14)
        card.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 6, 0 if column == 2 else 6))
        ttk.Label(card, text=title, style="Panel.TLabel").pack(anchor="w")
        value_label = ttk.Label(card, text=value, style="Value.Panel.TLabel")
        value_label.pack(anchor="w", pady=(5, 3))
        ttk.Label(card, text=detail, style="Muted.Panel.TLabel").pack(anchor="w")
        return value_label

    def _refresh_ports(self) -> None:
        current = self.port_var.get()
        devices = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = devices
        if current in devices:
            self.port_var.set(current)
        elif devices:
            self.port_var.set(devices[0])
        else:
            self.port_var.set("")
        self._refresh_motion_buttons()
        self._log(f"发现 {len(devices)} 个串口" + (f"：{', '.join(devices)}" if devices else ""))

    def _open_speed_window(self) -> None:
        if self.worker is None or not self.connected:
            return
        if self.speed_window is not None and self.speed_window.winfo_exists():
            self.speed_window.lift()
            self.speed_window.focus_force()
            return

        window = tk.Toplevel(self)
        self.speed_window = window
        self.speed_values_loaded = False
        window.title("速度设置")
        window.geometry("460x315")
        window.resizable(False, False)
        window.configure(bg=BG)
        window.transient(self)
        window.protocol("WM_DELETE_WINDOW", self._close_speed_window)

        panel = ttk.Frame(window, style="Panel.TFrame", padding=20)
        panel.pack(fill="both", expand=True, padx=14, pady=14)
        ttk.Label(panel, text="运行速度设置", style="Panel.TLabel",
                  font=("Microsoft YaHei UI", 15, "bold")).grid(
                      row=0, column=0, columnspan=3, sticky="w", pady=(0, 6)
                  )
        ttk.Label(panel, text="速度参数仅允许在电机停止时修改。", style="Muted.Panel.TLabel").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(0, 18)
        )

        self.forward_speed_var = tk.StringVar()
        self.reverse_speed_var = tk.StringVar()
        ttk.Label(panel, text="正转速度", style="Panel.TLabel").grid(row=2, column=0, sticky="w", pady=7)
        ttk.Entry(panel, textvariable=self.forward_speed_var, width=18).grid(row=2, column=1, sticky="ew", padx=12, pady=7)
        ttk.Label(panel, text="mm/s", style="Muted.Panel.TLabel").grid(row=2, column=2, sticky="w")
        ttk.Label(panel, text="反转速度", style="Panel.TLabel").grid(row=3, column=0, sticky="w", pady=7)
        ttk.Entry(panel, textvariable=self.reverse_speed_var, width=18).grid(row=3, column=1, sticky="ew", padx=12, pady=7)
        ttk.Label(panel, text="mm/s", style="Muted.Panel.TLabel").grid(row=3, column=2, sticky="w")

        self.speed_dialog_status = ttk.Label(panel, text="正在读取控制器速度参数…",
                                              style="Muted.Panel.TLabel")
        self.speed_dialog_status.grid(row=4, column=0, columnspan=3, sticky="w", pady=(14, 10))
        button_row = ttk.Frame(panel, style="Panel.TFrame")
        button_row.grid(row=5, column=0, columnspan=3, sticky="ew")
        button_row.columnconfigure((0, 1), weight=1, uniform="speed_button")
        ttk.Button(button_row, text="取消", command=self._close_speed_window).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        self.speed_apply_button = ttk.Button(button_row, text="应用设置", style="Home.TButton",
                                              command=self._apply_speeds, state="disabled")
        self.speed_apply_button.grid(row=0, column=1, sticky="ew", padx=(6, 0))
        panel.columnconfigure(1, weight=1)

        window.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - window.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - window.winfo_height()) // 2
        window.geometry(f"+{max(0, x)}+{max(0, y)}")
        self.worker.submit_read_speeds()

    def _close_speed_window(self) -> None:
        if self.speed_window is not None:
            try:
                self.speed_window.destroy()
            except tk.TclError:
                pass
        self.speed_window = None

    def _apply_speeds(self) -> None:
        if self.worker is None or not self.connected:
            messagebox.showerror("无法设置", "控制器尚未连接。")
            return
        if self.operation_busy:
            messagebox.showwarning("无法设置", "请等待当前运动完成或先停止运动。")
            return
        try:
            forward = int(self.forward_speed_var.get().strip())
            reverse = int(self.reverse_speed_var.get().strip())
            if not 1 <= forward <= 9_999_999 or not 1 <= reverse <= 9_999_999:
                raise ValueError
        except ValueError:
            messagebox.showerror("速度无效", "正转和反转速度必须是 1~9999999 之间的整数。")
            return
        self.speed_apply_button.configure(state="disabled")
        self.speed_dialog_status.configure(text="正在写入并校验速度参数…")
        self.worker.submit_set_speeds(forward, reverse)

    def _toggle_connection(self) -> None:
        if self.worker is not None:
            self._disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("未选择串口", "没有发现可用串口。请连接 USB-RS485 转换器后刷新。")
            return
        try:
            settings = ConnectionSettings(
                port=port,
                baudrate=int(self.baud_var.get()),
                slave_id=int(self.slave_var.get()),
                poll_interval=int(self.poll_var.get().split()[0]) / 1000,
                driver_subdivision=int(self.driver_subdivision_var.get()),
            )
            if not 1 <= settings.slave_id <= 255 or not 200 <= settings.driver_subdivision <= 60000:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "参数错误",
                "站号必须是1~255之间的整数，ISS35细分必须是200~60000之间的整数。",
            )
            return

        self.connect_button.configure(text="正在连接…", state="disabled")
        self.worker = ControllerWorker(settings, self.events)
        self.worker.start()
        self._log(
            f"正在连接 {settings.port}，{settings.baudrate} bps，站号 {settings.slave_id}；"
            f"准备校验ISS35细分 {settings.driver_subdivision} pulse/rev…"
        )

    def _disconnect(self) -> None:
        worker, self.worker = self.worker, None
        if worker is not None:
            worker.stop()
        self._set_connected(False)
        self._log("已请求断开连接")

    def _set_connected(self, connected: bool) -> None:
        self.connected = connected
        if not connected:
            self.calibrated = False
            self.operation_busy = False
            self.transmitter_zero_pending = False
            self.force_limit_pending = False
            self.force_limit_active = False
            self.force_limit_tripped = False
            self.last_force_poll_error = ""
            self.axis_lead_mm = None
            self.axis_subdivision = None
            self.pulses_per_mm = None
            self.position_offset_mm = 0.0
            self.force_value_label.configure(text="当前力：--")
            self.force_limit_status.configure(text="最大力保护未启用；比较正负力值的绝对值")
        self.status_dot.configure(fg=GREEN if connected else RED)
        self.status_text.configure(text="已连接" if connected else "未连接")
        self.connect_button.configure(text="断开" if connected else "连接", state="normal")
        self.driver_subdivision_spinbox.configure(
            state="disabled" if connected else "normal"
        )
        self.stop_button.configure(state="normal" if connected else "disabled")
        self._refresh_motion_buttons()

    def _refresh_motion_buttons(self) -> None:
        motion_available = self.connected and not self.operation_busy and not self.force_limit_tripped
        self.calibrate_button.configure(
            state="normal" if motion_available else "disabled"
        )
        self.home_button.configure(
            state="normal"
            if motion_available and self.calibrated
            else "disabled"
        )
        self.speed_button.configure(
            state="normal" if self.connected and not self.operation_busy else "disabled"
        )
        self.transmitter_zero_button.configure(
            state="normal"
            if self.connected and not self.transmitter_zero_pending
            else "disabled"
        )
        can_change_force_limit = self.connected and not self.operation_busy and not self.force_limit_pending
        self.force_limit_apply_button.configure(
            state="normal" if can_change_force_limit else "disabled"
        )
        self.force_limit_disable_button.configure(
            state="normal"
            if can_change_force_limit and (self.force_limit_active or self.force_limit_tripped)
            else "disabled"
        )
        self.transmitter_slave_spinbox.configure(
            state="disabled"
            if self.force_limit_active or self.force_limit_pending
            else "normal"
        )
        if self.speed_window is not None and self.speed_window.winfo_exists():
            can_apply = self.connected and not self.operation_busy and self.speed_values_loaded
            self.speed_apply_button.configure(state="normal" if can_apply else "disabled")
            if not self.connected:
                self.speed_dialog_status.configure(text="控制器已断开连接")

    def _send_stop(self) -> None:
        if self.worker is not None:
            self.worker.submit_emergency_stop("用户点击急停")
            self.operation_busy = False
            self._refresh_motion_buttons()
            self._log("急停已触发：已终止当前流程，正在发送停止线圈 0x0004…")

    def _send_transmitter_zero(self) -> None:
        if self.worker is None or not self.connected:
            return
        try:
            transmitter_slave = int(self.transmitter_slave_var.get().strip())
            if not 1 <= transmitter_slave <= 247:
                raise ValueError
        except ValueError:
            messagebox.showerror("站号无效", "变送器站号必须是 1~247 之间的整数。")
            return
        if transmitter_slave == self.worker.settings.slave_id:
            messagebox.showerror(
                "站号冲突",
                "变送器与运动控制器共用 RS485 总线，站号不能相同。",
            )
            return
        self.transmitter_zero_pending = True
        self.transmitter_zero_button.configure(state="disabled")
        self._log(
            f"正在通过 {self.worker.settings.port}、"
            f"{self.worker.settings.baudrate} bps 向变送器"
            f"站号 {transmitter_slave} 发送清零命令："
            "0x10 写寄存器 0x005E = 0x0001…"
        )
        self.worker.submit_transmitter_zero(transmitter_slave)

    def _apply_force_limit(self) -> None:
        if self.worker is None or not self.connected:
            return
        if self.operation_busy:
            messagebox.showwarning("无法修改", "机构运动时不能修改最大力保护。")
            return
        try:
            transmitter_slave = int(self.transmitter_slave_var.get().strip())
            maximum_force = float(self.maximum_force_var.get().strip())
            if not 1 <= transmitter_slave <= 247:
                raise ValueError
            if not math.isfinite(maximum_force) or maximum_force <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "最大力无效",
                "请输入大于 0 的有限数值，并确认变送器站号为 1~247。",
            )
            return
        if transmitter_slave == self.worker.settings.slave_id:
            messagebox.showerror("站号冲突", "变送器站号不能与 FUYU 站号相同。")
            return
        self.force_limit_pending = True
        self._refresh_motion_buttons()
        self.force_limit_status.configure(text="正在读取变送器小数位并启用保护…")
        self.worker.submit_force_limit(transmitter_slave, maximum_force)

    def _disable_force_limit(self) -> None:
        if self.worker is None or not self.connected or self.operation_busy:
            return
        self.force_limit_pending = True
        self._refresh_motion_buttons()
        self.worker.submit_disable_force_limit()

    def _send_calibration(self) -> None:
        if self.worker is None:
            return
        confirmed = messagebox.askyesno(
            "确认标定原点",
            "标定将先反转向上运动；控制器触发物理正限位并停止后，再正转向下移动 50 mm。\n\n"
            "请确认物理限位、运动方向和工作区域均安全。是否开始？",
            icon="warning",
        )
        if not confirmed:
            return
        self.calibrated = False
        self.operation_busy = True
        self._refresh_motion_buttons()
        self.worker.submit_calibration()
        self._log("开始标定原点：反转向上，等待控制器触发物理正限位并停止")

    def _send_home(self) -> None:
        if self.worker is None or not self.calibrated:
            return
        if messagebox.askyesno("确认启动回零", "将通过绝对运动返回本次标定的 0 mm 原点。是否开始？", icon="warning"):
            self.operation_busy = True
            self._refresh_motion_buttons()
            self.worker.submit_home()
            self._log("正在返回已标定的 0 mm 原点…")

    def _display_values(self, values: MonitorValues) -> None:
        if not self.pulses_per_mm:
            return
        position_mm = (
            POSITION_DIRECTION * values.position / self.pulses_per_mm
            + self.position_offset_mm
        )
        self.position_value.configure(text=f"{position_mm:,.2f}")
        self.speed_value.configure(text=f"{values.speed:,}")
        self.input_value.configure(text=f"0x{values.inputs:08X}")
        changed = 0 if self.previous_inputs is None else self.previous_inputs ^ values.inputs
        if changed:
            changed_bits = ", ".join(str(bit) for bit in range(32) if changed & (1 << bit))
            self._log(f"输入状态变化：BIT {changed_bits}，当前值 0x{values.inputs:08X}")
        self.previous_inputs = values.inputs

    def _process_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "connected":
                    self._set_connected(True)
                    self._log(f"连接成功：{payload.port}")
                elif kind == "axis_config_corrected":
                    previous, corrected = payload
                    self._log(
                        f"运动换算已修正：FSC-2A细分由 {previous} 改为 {corrected} pulse/rev，"
                        "并已回读确认"
                    )
                elif kind == "axis_config":
                    lead, subdivision, pulses_per_mm = payload
                    self.axis_lead_mm = lead
                    self.axis_subdivision = subdivision
                    self.pulses_per_mm = pulses_per_mm
                    self._log(
                        f"控制器参数：导程 {lead} mm，细分 {subdivision} pulse，"
                        f"位置换算 {pulses_per_mm:g} 计数/mm "
                        f"({1 / pulses_per_mm:g} mm/计数)"
                    )
                elif kind == "position_reference":
                    offset_mm, display_mm = payload
                    self.position_offset_mm = offset_mm
                    self.position_value.configure(text=f"{display_mm:,.2f}")
                elif kind == "values":
                    self._display_values(payload)
                    self.last_poll_error = ""
                elif kind == "command_ok":
                    self._log(f"{payload}命令发送成功")
                    if payload == "变送器清零":
                        self.transmitter_zero_pending = False
                        self.transmitter_zero_button.configure(
                            state="normal" if self.connected else "disabled"
                        )
                elif kind == "command_error":
                    name, error = payload
                    if name not in ("变送器清零", "设置最大力保护"):
                        self.operation_busy = False
                        self._refresh_motion_buttons()
                    if name in ("读取速度", "设置速度") and self.speed_window is not None:
                        self.speed_dialog_status.configure(text=f"{name}失败：{error}")
                        if name == "设置速度" and self.speed_values_loaded:
                            self.speed_apply_button.configure(state="normal")
                    if name == "变送器清零":
                        self.transmitter_zero_pending = False
                        self.transmitter_zero_button.configure(
                            state="normal" if self.connected else "disabled"
                        )
                    if name == "设置最大力保护":
                        self.force_limit_pending = False
                        self.force_limit_status.configure(text=f"最大力保护启用失败：{error}")
                        self._refresh_motion_buttons()
                    self._log(f"{name}命令失败：{error}")
                    if name == "变送器清零":
                        messagebox.showerror(
                            "变送器清零失败",
                            f"变送器没有确认清零命令：\n{error}\n\n"
                            "请核对变送器是否为 115200/8N1、站号及 A/B 接线。",
                        )
                    elif name == "设置最大力保护":
                        messagebox.showerror(
                            "最大力保护启用失败",
                            f"无法读取变送器状态：\n{error}",
                        )
                    else:
                        messagebox.showerror("命令失败", f"{name}命令未被控制器确认：\n{error}")
                elif kind == "speed_values":
                    forward, reverse = payload
                    if self.speed_window is not None and self.speed_window.winfo_exists():
                        self.forward_speed_var.set(str(forward))
                        self.reverse_speed_var.set(str(reverse))
                        self.speed_values_loaded = True
                        self.speed_dialog_status.configure(
                            text="已读取：正转 0x0005/06，反转 0x0027/28"
                        )
                        self.speed_apply_button.configure(
                            state="normal" if self.connected and not self.operation_busy else "disabled"
                        )
                    self._log(f"已读取速度：正转 {forward} mm/s，反转 {reverse} mm/s")
                elif kind == "speed_set_ok":
                    forward, reverse = payload
                    if self.speed_window is not None and self.speed_window.winfo_exists():
                        self.speed_dialog_status.configure(text="速度参数设置成功")
                        self.speed_apply_button.configure(
                            state="normal" if self.connected and not self.operation_busy else "disabled"
                        )
                    self._log(f"速度设置成功：正转 {forward} mm/s，反转 {reverse} mm/s")
                elif kind == "force_limit_configured":
                    maximum_force, decimal_places, force_slave = payload
                    self.force_limit_pending = False
                    self.force_limit_active = True
                    self.force_limit_tripped = False
                    self.last_force_poll_error = ""
                    self.force_limit_status.configure(
                        text=(
                            f"最大力保护已启用：|F| > {maximum_force:g} 时停止；"
                            f"变送器站号 {force_slave}，小数位 {decimal_places}"
                        )
                    )
                    self._refresh_motion_buttons()
                    self._log(
                        f"最大力保护已启用：绝对力值超过 {maximum_force:g} 时立即发送停止命令"
                    )
                elif kind == "force_limit_disabled":
                    self.force_limit_pending = False
                    self.force_limit_active = False
                    self.force_limit_tripped = False
                    self.last_force_poll_error = ""
                    self.force_value_label.configure(text="当前力：--")
                    self.force_limit_status.configure(text="最大力保护未启用；比较正负力值的绝对值")
                    self._refresh_motion_buttons()
                    self._log("最大力保护已停用")
                elif kind == "force_value":
                    force, decimal_places = payload
                    self.force_value_label.configure(
                        text=f"当前力：{force:,.{decimal_places}f}"
                    )
                    self.last_force_poll_error = ""
                elif kind == "force_limit_tripped":
                    force, maximum_force, interrupted_kind, stop_error = payload
                    self.operation_busy = False
                    self.force_limit_tripped = True
                    if interrupted_kind == "calibration":
                        self.calibrated = False
                    self._refresh_motion_buttons()
                    if stop_error is None:
                        stop_text = "FUYU 已确认停止命令"
                        status_text = "运动已停止"
                    else:
                        stop_text = f"停止命令重试 {STOP_RETRY_COUNT} 次仍未收到确认：{stop_error}"
                        status_text = "停止命令未确认"
                    self.force_limit_status.configure(
                        text=f"超力保护已触发：|{force:g}| > {maximum_force:g}；{status_text}，请重新布防"
                    )
                    self._log(
                        f"超力急停：当前力 {force:g}，最大允许力 {maximum_force:g}；{stop_text}"
                    )
                    messagebox.showerror(
                        "超力停止",
                        f"当前力 {force:g} 已超过最大力 {maximum_force:g}。\n"
                        f"{stop_text}。排除原因后请点击“应用并启用”重新布防。",
                    )
                elif kind == "emergency_stop_result":
                    reason, interrupted_kind, stop_error = payload
                    self.operation_busy = False
                    if interrupted_kind == "calibration":
                        self.calibrated = False
                    self._refresh_motion_buttons()
                    if stop_error is None:
                        self._log(f"{reason}：FUYU 已确认停止命令")
                    else:
                        self._log(
                            f"{reason}：停止命令重试 {STOP_RETRY_COUNT} 次仍未收到确认：{stop_error}"
                        )
                        messagebox.showerror(
                            "急停命令未确认",
                            f"当前运动流程已锁定，但 FUYU 未确认停止命令。\n{stop_error}",
                        )
                elif kind == "force_poll_error":
                    error, count = payload
                    if error != self.last_force_poll_error or count in (1, 3, 10):
                        self._log(f"变送器力值读取失败（连续 {count} 次）：{error}")
                    self.last_force_poll_error = error
                elif kind == "force_monitor_failed":
                    error, interrupted_kind, stop_error = payload
                    self.operation_busy = False
                    self.force_limit_tripped = True
                    if interrupted_kind == "calibration":
                        self.calibrated = False
                    self._refresh_motion_buttons()
                    stop_text = "已发送停止命令" if stop_error is None else f"停止命令也失败：{stop_error}"
                    self.force_limit_status.configure(text="力传感器通信中断；保护已锁定")
                    self._log(f"力监控通信中断，{stop_text}：{error}")
                    messagebox.showerror(
                        "力监控通信中断",
                        f"连续 3 次无法读取力值，{stop_text}。\n{error}",
                    )
                elif kind == "operation_step":
                    self._log(payload)
                elif kind == "operation_cancelled":
                    self.operation_busy = False
                    self._refresh_motion_buttons()
                    self._log(payload)
                elif kind == "operation_error":
                    self.operation_busy = False
                    self.calibrated = False
                    self._refresh_motion_buttons()
                    self._log(f"动作失败：{payload}")
                    messagebox.showerror("动作失败", payload)
                elif kind == "calibration_done":
                    self.operation_busy = False
                    self.calibrated = True
                    self._refresh_motion_buttons()
                    self._log(f"原点标定完成：已从正限位向下移动 {payload:.3f} mm，并将当前位置设为 0")
                    messagebox.showinfo("标定完成", "原点标定完成，现在可以点击“启动回零”。")
                elif kind == "home_done":
                    self.operation_busy = False
                    self._refresh_motion_buttons()
                    position_mm = (
                        POSITION_DIRECTION * payload / self.pulses_per_mm
                        if self.pulses_per_mm
                        else 0.0
                    )
                    self._log(f"回零完成：当前位置 {position_mm:.2f} mm")
                elif kind == "poll_error":
                    error, count = payload
                    if error != self.last_poll_error or count in (1, 5, 20):
                        self._log(f"读取失败（连续 {count} 次）：{error}")
                    self.last_poll_error = error
                elif kind == "connect_error":
                    self._log(f"连接失败：{payload}")
                    messagebox.showerror("连接失败", payload)
                elif kind == "disconnected":
                    # Ignore a late event from an old worker if the user has
                    # already started a new connection.
                    if self.worker is payload:
                        self.worker = None
                        self._set_connected(False)
                    elif self.worker is None:
                        self._set_connected(False)
        except queue.Empty:
            pass
        self.after(50, self._process_events)

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log.insert("end", f"[{timestamp}] {message}\n")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 300:
            self.log.delete("1.0", "51.0")
        self.log.see("end")

    def _on_close(self) -> None:
        if self.worker is not None:
            self.worker.stop()
        self.destroy()


if __name__ == "__main__":
    FSC2AApp().mainloop()

