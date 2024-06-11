import pytest
from picarx.interfaces.actuators import TravelDirection, MotorSide
from picarx.drivers.dcmotor import AbstractDCMotorDriver

class TestAbstractDCMotorDriver:
    @pytest.fixture
    def dcmotor(self):
        return AbstractDCMotorDriver("test_motor", 1, 2, motor_side=MotorSide.LEFT)

    def test_name(self, dcmotor):
        assert dcmotor.name == "test_motor"

    def test_direction_pin(self, dcmotor):
        assert dcmotor.direction_pin == 1

    def test_pwm_pin(self, dcmotor):
        assert dcmotor.pwm_pin == {'channel': 2, 'i2c_port': '\dev\i2c-1'}

    def test_motor_side(self, dcmotor):
        assert dcmotor.motor_side == MotorSide.LEFT

    def test_direction(self, dcmotor):
        assert dcmotor.direction == TravelDirection.FORWARD

    def test_speed(self, dcmotor):
        assert dcmotor.speed is None

    def test_drive_with_speed(self, dcmotor):
        dcmotor.drive_with_speed(50)
        assert dcmotor.speed == 50

    def test_start(self, dcmotor):
        with pytest.raises(NotImplementedError):
            dcmotor.start()

    def test_stop(self, dcmotor):
        with pytest.raises(NotImplementedError):
            dcmotor.stop()