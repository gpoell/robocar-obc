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
        self.assertTrue(hasattr(self.motor, 'pwm'), "Motor class should have a 'pwm' property")
        self.assertTrue(hasattr(self.motor, 'adc'), "Motor class should have a 'adc' property")
        self.assertTrue(hasattr(self.motor, 'target_distance'), "Motor class should have a 'target_distance' property")

    def test_motor_property_values(self):
        """Test to ensure Motor class is instantiated with correct property types and values."""

        wrong_rotation_delay = "2.5"
        wrong_pwm_address = 4.5
        wrong_pwm_freq = "5000"

        self.assertRaises(TypeError, Motor, wrong_rotation_delay)
        self.assertRaises(TypeError, Motor, wrong_pwm_address)
        self.assertRaises(TypeError, Motor, wrong_pwm_freq)

    def test_duty_range(self):
        """Test the duty range method to ensure it returns tuple values within 12-bit (4905) resolution"""

        test = self.motor.duty_range(5000, 40, -595, -5000)
        expected = (4095, 40, -595, -4095)

        self.assertIs(test, expected, "The duty range test values are not (4095, 40, -595, -4095)")