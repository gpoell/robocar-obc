import unittest
from client.topics import UltrasonicPublishers, UltrasonicSubscribers, Topics
from client.mqtt_client import ClientConfig, Environment
from parts.ultrasonic import Ultrasonic

class TestUltrasonic(unittest.TestCase):
    """Tests for the Ultrasonic class."""

    def setUp(self):

        # Create Ultrasonic instance
        publishers = UltrasonicPublishers()
        subscribers = UltrasonicSubscribers()
        topics = Topics(publishers, subscribers)
        config = ClientConfig(topics=topics, host='mqtt-broker', port=1883)

        # Use default parameters
        self.device = Ultrasonic(client_config=config)

    def tearDown(self):
        self.device.stop()


    def test_ultrasonic_property_types(self):
        """Test for correct property types for the Ultrasonic class."""

        self.assertIsInstance(self.device.trigger_pin, int)
        self.assertIsInstance(self.device.echo_pin, int)
        self.assertIsInstance(self.device.speed_conversion, float)


    def test_ultrasonic_property_exists(self):
        """Test to ensure Ultrasonic class has properties: trigger_pin, echo_pin, timeout, GPIO"""

        self.assertTrue(hasattr(self.device, 'trigger_pin'), "Ultrasonic should have a 'trigger_pin' property")
        self.assertTrue(hasattr(self.device, 'echo_pin'), "Ultrasonic should have a 'echo_pin' property")
        # self.assertTrue(hasattr(self.device, 'GPIO'), "Ultrasonic should have a 'GPIO' property")
        self.assertTrue(hasattr(self.device, 'speed_conversion'), "Ultrasonic should have a 'speed_conversion' property")

    def test_gpio_setup(self):
        """
        This test covers GPIO setup at initialization depending on execution environment.
        Important for interfacing with physical hardware or running in a disconnected mode.
        """
        if self.device.env == Environment.PROD:
            trig_stat = self.device.GPIO.gpio_function(self.device.trigger_pin)
            echo_stat = self.device.GPIO.gpio_function(self.device.echo_pin)

            self.assertIs(trig_stat, 0)
            self.assertIs(echo_stat, 1)


    def test_calculate_pulse_time(self):
        """
        Test to ensure pulse time returns correct values. This should only be tested
        when running in PROD on the Raspberry Pi.
        """

        if self.device.env == Environment.PROD:
            result = self.device.calculate_pulse_time()
            self.assertIsInstance(result, float)


    def test_calculate_distance(self):
        """Test to ensure correct type of value is returned when calculating the distance."""

        result = self.device.calculate_distance()
        self.assertIsInstance(result, float)


if __name__ == '__main__':
    unittest.main()