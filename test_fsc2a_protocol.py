import struct
import unittest

from fsc2a_protocol import (
    ModbusRTUClient,
    add_crc,
    crc16_modbus,
    int32_from_registers,
    uint32_from_registers,
)


class FakeSerial:
    def __init__(self, response: bytes):
        self.response = bytearray(response)
        self.written = b""
        self.timeout = 0.1
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.response)

    def read(self, size=1):
        result = bytes(self.response[:size])
        del self.response[:size]
        return result

    def write(self, data):
        self.written += data
        return len(data)

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


class ProtocolTests(unittest.TestCase):
    def test_crc_matches_manual_read_example(self):
        payload = bytes.fromhex("01 03 00 01 00 02")
        self.assertEqual(add_crc(payload), bytes.fromhex("01 03 00 01 00 02 95 CB"))
        self.assertEqual(crc16_modbus(payload), 0xCB95)

    def test_crc_matches_manual_coil_example(self):
        payload = bytes.fromhex("01 05 00 01 FF 00")
        self.assertEqual(add_crc(payload), bytes.fromhex("01 05 00 01 FF 00 DD FA"))

    def test_signed_and_unsigned_register_join(self):
        self.assertEqual(int32_from_registers(0x0001, 0x0002), 65538)
        self.assertEqual(int32_from_registers(0xFFFF, 0xFF9C), -100)
        self.assertEqual(uint32_from_registers(0xFFFF, 0xFF9C), 0xFFFFFF9C)

    def test_read_monitor_values(self):
        data = struct.pack(">6H", 0xFFFF, 0xFF9C, 0x0000, 0x0032, 0x0000, 0x0005)
        response = add_crc(bytes((1, 3, len(data))) + data)
        serial_port = FakeSerial(response)
        client = ModbusRTUClient(serial_port, minimum_interval=0.020)
        values = client.read_monitor_values()
        self.assertEqual(values.position, -100)
        self.assertEqual(values.speed, 50)
        self.assertEqual(values.inputs, 5)
        self.assertEqual(serial_port.written, bytes.fromhex("01 03 00 4A 00 06 E4 1E"))

    def test_write_stop_coil(self):
        response = bytes.fromhex("01 05 00 04 FF 00 CD FB")
        serial_port = FakeSerial(response)
        client = ModbusRTUClient(serial_port)
        client.write_single_coil(0x0004)
        self.assertEqual(serial_port.written, response)

    def test_write_32_bit_move_distance(self):
        response = add_crc(bytes.fromhex("01 10 00 0F 00 02"))
        serial_port = FakeSerial(response)
        client = ModbusRTUClient(serial_port)
        client.write_int32(0x000F, 50)
        expected_request = add_crc(bytes.fromhex("01 10 00 0F 00 02 04 00 00 00 32"))
        self.assertEqual(serial_port.written, expected_request)

    def test_write_pressure_transmitter_manual_zero_matches_manual(self):
        response = bytes.fromhex("01 10 00 5E 00 01 60 1B")
        serial_port = FakeSerial(response)
        client = ModbusRTUClient(serial_port)
        client.write_multiple_registers(0x005E, [0x0001])
        self.assertEqual(
            serial_port.written,
            bytes.fromhex("01 10 00 5E 00 01 02 00 01 6A EE"),
        )

    def test_write_registers_can_target_another_bus_slave(self):
        response = add_crc(bytes.fromhex("02 10 00 5E 00 01"))
        serial_port = FakeSerial(response)
        client = ModbusRTUClient(serial_port, slave_id=1)
        client.write_multiple_registers(0x005E, [0x0001], slave_id=2)
        self.assertEqual(
            serial_port.written,
            add_crc(bytes.fromhex("02 10 00 5E 00 01 02 00 01")),
        )

    def test_read_registers_can_target_another_bus_slave(self):
        data = struct.pack(">2H", 0xFFFF, 0xFF9C)
        response = add_crc(bytes((2, 3, len(data))) + data)
        serial_port = FakeSerial(response)
        client = ModbusRTUClient(serial_port, slave_id=1)
        registers = client.read_holding_registers(0x0050, 2, slave_id=2)
        self.assertEqual(registers, [0xFFFF, 0xFF9C])
        self.assertEqual(
            serial_port.written,
            add_crc(bytes.fromhex("02 03 00 50 00 02")),
        )


if __name__ == "__main__":
    unittest.main()

