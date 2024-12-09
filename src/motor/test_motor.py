import unittest
from motor.motor import Motor

class TestMotor(unittest.TestCase):
    """
    Test cases for the Motor class. These tests must be run in PROD because
    the dependencies require a Raspberry Pi.
    """

    def setUp(self) -> None:

        self.motor = Motor(rotation_delay=2.5)

    def tearDown(self) -> None:
        pass

    def test_motor_property_exists(self):
        """Test to ensure Motor class contains the properties: rotation_delay, pwm, adc"""

        self.assertTrue(hasattr(self.motor, 'rotation_delay'), "Motor class should have a 'rotation_delay' property")
        self.assertTrue(hasattr(self.motor, 'pwm_address'), "Motor class should have a 'pwm_address' property")
        self.assertTrue(hasattr(self.motor, 'pwm_frequency'), "Motor class should have a 'pwm_frequency' property")
        self.assertTrue(hasattr(self.motor, 'resolution'), "Motor class should have a 'resolution' property")

    def test_motor_property_values(self):
        """Test to ensure Motor class is instantiated with correct property types and values."""

        wrong_rotation_delay = "2.5"
        wrong_pwm_address = 4.5
        wrong_pwm_freq = "5000"

        self.assertRaises(TypeError, Motor, wrong_rotation_delay)
        self.assertRaises(TypeError, Motor, wrong_pwm_address)
        self.assertRaises(TypeError, Motor, wrong_pwm_freq)
