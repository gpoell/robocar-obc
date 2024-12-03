import unittest
import os

# def add_nums(num1, num2) -> int:
#     return num1 + num2

# class TestMain(unittest.TestCase):

#     def test_add_numbers(self):
#         self.assertEqual(add_nums(1, 1), 2)

#     def test_add_strings(self):
#         self.assertEqual(add_nums('a','a'), 'aa')


class TestUltrasonic(unittest.TestCase):
    """Test class for ultrasonic module and methods"""

    def setUp(self) -> None:
        # set OS environ
        return super().setUp()

    def test_class_init(self):
        pass

    def test_gpio_setup(self):
        """
        This test covers GPIO setup at initialization depending on execution environment.
        Important for interfacing with physical hardware or running in a disconnected mode.
        """
        # Set environ == DEV
        # Set environ == PROD
        pass

    def test_client_on_connect(self):
        """Tests successful connection to MQTT broker."""
        # If environ == DEV: return true
        # If environ == PROD: execute method
        pass

    def test_client_on_message(self):
        """Test case for when the ultrasonic sensor receives messages from the MQTT broker"""
        # If environ == DEV: use Mock client to publish message
        # If environ == PROD: use active broker to send message
        pass

    def

if __name__ == '__main__':
    unittest.main()