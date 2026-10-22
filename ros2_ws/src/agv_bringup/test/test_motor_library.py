"""Exercise the copied library without importing PyModbus or opening a port."""
from types import SimpleNamespace

import pytest

from agv_bringup.ZLAC8015D import Controller


class FakeClient:
    def __init__(self):
        self.registers = {}
        self.failed = False
        self.reads = 0
        self.closed = False

    def connect(self):
        return True

    def close(self):
        self.closed = True

    def read_holding_registers(self, address, count, slave):
        self.reads += 1
        if self.failed and self.reads > 5:
            raise AssertionError('Reads must not retry forever')
        return SimpleNamespace(isError=lambda: self.failed,
                               registers=[self.registers.get(address+i, 0) for i in range(count)])

    def write_register(self, address, value, slave):
        self.registers[address] = value
        return SimpleNamespace(isError=lambda: self.failed)

    def write_registers(self, address, values, slave):
        self.registers.update({address+i: value for i, value in enumerate(values)})
        return SimpleNamespace(isError=lambda: self.failed)


def test_existing_motor_functions_write_signed_rpm_and_stop():
    client = FakeClient()
    motors = Controller(client=client)
    motors.set_mode(3)
    motors.set_rpm(-10, 12)
    assert client.registers[0x200D] == 3
    assert client.registers[0x2088] == 65526
    assert client.registers[0x2089] == 12
    motors.disable_motor()
    assert client.registers[0x200E] == 7


def test_failed_read_raises_once_instead_of_looping():
    client = FakeClient()
    motors = Controller(client=client)
    client.failed = True
    with pytest.raises(IOError):
        motors.get_rpm()
    assert client.reads == 1


def test_failed_write_is_not_silently_ignored():
    client = FakeClient()
    motors = Controller(client=client)
    client.failed = True
    with pytest.raises(IOError):
        motors.set_rpm(10, -10)


def test_combined_faults_and_signed_encoder_feedback():
    client = FakeClient()
    client.registers.update({0x20A5: 0x0820, 0x20A6: 0,
                             0x20A7: 0xFFFF, 0x20A8: 0xFFFE,
                             0x20A9: 0, 0x20AA: 2})
    motors = Controller(client=client)
    assert motors.get_fault_code() == ((True, 0x0820), (False, 0))
    assert motors.get_wheels_tick() == (-2, 2)


def test_device_watchdog_is_written_and_read_back():
    client = FakeClient()
    motors = Controller(client=client)
    motors.set_communication_timeout(500)
    assert client.registers[0x2000] == 500
    with pytest.raises(ValueError):
        motors.set_communication_timeout(0)
