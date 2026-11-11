"""FSC-2A Modbus RTU protocol support.

The controller manual specifies function codes 0x03 and 0x05 for the
operations used by this application.  This module deliberately implements
only that small, well-tested subset instead of pulling in a full Modbus stack.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import Protocol


READ_POSITION = 0x004A
READ_SPEED = 0x004C
READ_INPUTS = 0x004E
AXIS_LEAD = 0x0001
AXIS_SUBDIVISION = 0x0003
SPEED_FORWARD = 0x0005
SPEED_REVERSE = 0x0027
MOVE_DISTANCE = 0x000F
COIL_RELATIVE_FORWARD = 0x0001
COIL_RELATIVE_REVERSE = 0x0002
COIL_ABSOLUTE = 0x0003
COIL_STOP = 0x0004
COIL_HOME = 0x000B
COIL_SET_POSITION_ZERO = 0x000D
TRANSMITTER_MANUAL_ZERO = 0x005E
TRANSMITTER_STATUS = 0x0008
TRANSMITTER_GROSS_FORCE = 0x0050


class SerialPort(Protocol):
    timeout: float | None

    @property
    def in_waiting(self) -> int: ...

    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


class FSC2AError(Exception):
    """Base exception for controller communication errors."""


class FSC2ATimeout(FSC2AError):
    """The controller did not return a complete response in time."""


class FSC2AProtocolError(FSC2AError):
    """The controller returned an invalid or exception response."""


def crc16_modbus(data: bytes) -> int:
    """Return the Modbus CRC-16 value for *data*."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def add_crc(payload: bytes) -> bytes:
    """Append Modbus's low-byte-first CRC to a payload."""
    return payload + struct.pack("<H", crc16_modbus(payload))


def int32_from_registers(high: int, low: int) -> int:
    """Join high/low 16-bit registers and decode two's-complement int32."""
    unsigned = ((high & 0xFFFF) << 16) | (low & 0xFFFF)
    return unsigned - 0x1_0000_0000 if unsigned & 0x8000_0000 else unsigned


def uint32_from_registers(high: int, low: int) -> int:
    """Join high/low 16-bit registers into an unsigned 32-bit integer."""
    return ((high & 0xFFFF) << 16) | (low & 0xFFFF)


@dataclass(frozen=True)
class MonitorValues:
    position: int
    speed: int
    inputs: int


class ModbusRTUClient:
    """Minimal synchronous Modbus RTU client for one FSC-2A controller."""

    def __init__(
        self,
        serial_port: SerialPort,
        slave_id: int = 1,
        *,
        minimum_interval: float = 0.020,
    ) -> None:
        if not 1 <= slave_id <= 255:
            raise ValueError("从机地址必须在 1~255 之间")
        self.serial = serial_port
        self.slave_id = slave_id
        self.minimum_interval = max(0.020, minimum_interval)
        self._last_transaction_ended_at = 0.0
        self._lock = threading.Lock()

    def close(self) -> None:
        self.serial.close()

    def _wait_for_interval(self) -> None:
        delay = self.minimum_interval - (
            time.monotonic() - self._last_transaction_ended_at
        )
        if delay > 0:
            time.sleep(delay)

    def _mark_transaction_end(self) -> None:
        self._last_transaction_ended_at = time.monotonic()

    def _read_exactly(self, count: int) -> bytes:
        result = bytearray()
        while len(result) < count:
            chunk = self.serial.read(count - len(result))
            if not chunk:
                self._mark_transaction_end()
                raise FSC2ATimeout(
                    f"等待 {count} 字节响应超时，仅收到 {len(result)} 字节"
                )
            result.extend(chunk)
        return bytes(result)

    @staticmethod
    def _validate_crc(frame: bytes) -> None:
        if len(frame) < 4:
            raise FSC2AProtocolError("响应帧过短")
        expected = crc16_modbus(frame[:-2])
        received = struct.unpack("<H", frame[-2:])[0]
        if expected != received:
            raise FSC2AProtocolError(
                f"CRC 错误：收到 0x{received:04X}，应为 0x{expected:04X}"
            )

    def _begin_request(self, frame: bytes) -> None:
        self._wait_for_interval()
        self.serial.reset_input_buffer()
        written = self.serial.write(frame)
        if written != len(frame):
            self._mark_transaction_end()
            raise FSC2AProtocolError(f"串口仅发送 {written}/{len(frame)} 字节")

    def read_holding_registers(
        self,
        address: int,
        count: int,
        *,
        slave_id: int | None = None,
    ) -> list[int]:
        if not 0 <= address <= 0xFFFF:
            raise ValueError("寄存器地址超出范围")
        if not 1 <= count <= 125:
            raise ValueError("读取寄存器数量必须在 1~125 之间")

        target_slave = self.slave_id if slave_id is None else slave_id
        if not 1 <= target_slave <= 255:
            raise ValueError("从机地址必须在 1~255 之间")

        request = add_crc(struct.pack(">BBHH", target_slave, 0x03, address, count))
        with self._lock:
            self._begin_request(request)
            header = self._read_exactly(3)
            slave, function, third = header
            if function & 0x80:
                tail = self._read_exactly(2)
                frame = header + tail
                self._mark_transaction_end()
                self._validate_crc(frame)
                raise FSC2AProtocolError(f"控制器返回 Modbus 异常码 0x{third:02X}")
            if slave != target_slave or function != 0x03:
                self._mark_transaction_end()
                raise FSC2AProtocolError(
                    f"响应头不匹配：站号 {slave}，功能码 0x{function:02X}"
                )
            expected_bytes = count * 2
            if third != expected_bytes:
                self._mark_transaction_end()
                raise FSC2AProtocolError(
                    f"响应数据长度为 {third}，期望 {expected_bytes}"
                )
            frame = header + self._read_exactly(third + 2)
            self._mark_transaction_end()
            self._validate_crc(frame)
            return list(struct.unpack(f">{count}H", frame[3:-2]))

    def write_single_coil(self, address: int, enabled: bool = True) -> None:
        if not 0 <= address <= 0xFFFF:
            raise ValueError("线圈地址超出范围")
        value = 0xFF00 if enabled else 0x0000
        request = add_crc(struct.pack(">BBHH", self.slave_id, 0x05, address, value))
        with self._lock:
            self._begin_request(request)
            response = self._read_exactly(8)
            self._mark_transaction_end()
            self._validate_crc(response)
            if response != request:
                raise FSC2AProtocolError(
                    f"写线圈应答与请求不一致：{response.hex(' ').upper()}"
                )

    def write_multiple_registers(
        self,
        address: int,
        values: list[int],
        *,
        slave_id: int | None = None,
    ) -> None:
        """Write consecutive 16-bit registers with Modbus function 0x10."""
        if not 0 <= address <= 0xFFFF:
            raise ValueError("寄存器地址超出范围")
        if not 1 <= len(values) <= 123:
            raise ValueError("写入寄存器数量必须在 1~123 之间")
        if any(not 0 <= value <= 0xFFFF for value in values):
            raise ValueError("寄存器值必须在 0~65535 之间")

        target_slave = self.slave_id if slave_id is None else slave_id
        if not 1 <= target_slave <= 255:
            raise ValueError("从机地址必须在 1~255 之间")

        count = len(values)
        data = struct.pack(f">{count}H", *values)
        payload = struct.pack(">BBHHB", target_slave, 0x10, address, count, len(data)) + data
        request = add_crc(payload)
        with self._lock:
            self._begin_request(request)
            response = self._read_exactly(8)
            self._mark_transaction_end()
            self._validate_crc(response)
            slave, function, response_address, response_count = struct.unpack(">BBHH", response[:-2])
            if function & 0x80:
                raise FSC2AProtocolError("控制器返回写寄存器异常响应")
            if (slave, function, response_address, response_count) != (
                target_slave,
                0x10,
                address,
                count,
            ):
                raise FSC2AProtocolError(
                    f"写寄存器应答不匹配：{response.hex(' ').upper()}"
                )

    def write_int32(self, address: int, value: int) -> None:
        """Write a signed/unsigned 32-bit value as high then low register."""
        if not -0x8000_0000 <= value <= 0xFFFF_FFFF:
            raise ValueError("32 位写入值超出范围")
        unsigned = value & 0xFFFF_FFFF
        self.write_multiple_registers(address, [unsigned >> 16, unsigned & 0xFFFF])

    def read_monitor_values(self) -> MonitorValues:
        """Read 0x004A..0x004F in one atomic Modbus transaction."""
        registers = self.read_holding_registers(READ_POSITION, 6)
        return MonitorValues(
            position=int32_from_registers(registers[0], registers[1]),
            speed=int32_from_registers(registers[2], registers[3]),
            inputs=uint32_from_registers(registers[4], registers[5]),
        )

