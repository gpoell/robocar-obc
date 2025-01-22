import unittest
from motor import Motor
from PCA9685 import PCA9685

class TestMotor(unittest.TestCase):
    """
    Test cases for the Motor class. These tests must be run in PROD because
    the dependencies require a Raspberry Pi.
    """

    def setUp(self) -> None:

        self.motor = Motor()
        self.pwm = PCA9685()

    def tearDown(self) -> None:
        pass

    def test_motor_property_exists(self):
        """Test to ensure Motor class contains the properties: rotation_delay, pwm, adc"""

        self.assertTrue(hasattr(self.motor, 'pwm'), "Motor class should have a 'pwm' property")

    def test_motor_property_values(self):
        """Test to ensure Motor class is instantiated with correct property types and values."""

        wrong_pwm_address = 4.5
        wrong_pwm_freq = "5000"

        self.assertRaises(TypeError, Motor, wrong_pwm_address)
        self.assertRaises(TypeError, Motor, wrong_pwm_freq)

    def test_duty_range(self):
        """Test the duty range method to ensure it returns tuple values within 12-bit (4905) resolution"""


        test_positive = self.pwm.duty_range(5000)
        positive_expected = self.pwm.resolution
        test_negative = self.pwm.duty_range(-5000)
        negative_expected = -self.pwm.resolution

        self.assertIs(test_positive, positive_expected, "The positive duty range test value is not 4095.")
        self.assertEqual(test_negative, negative_expected, "The negative duty range test value is not -4095.")

if __name__ == '__main__':
    unittest.main()