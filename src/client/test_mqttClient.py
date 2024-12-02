import unittest
from topics import Topics, Topic, Publishers, Subscribers, UltrasonicPublishers, UltrasonicSubscribers
from mqtt_client import ClientConfig, MqttDevice, Connection

class TestTopics(unittest.TestCase):
    """
    Test class for MQTT Client data types.

    [IMPORTANT]: Run these tests within a docker container and with an active mqtt-broker container.
        Start the broker with `docker compose up -d mqtt-broker` (see docker-compose.yaml).
    """

    def setUp(self) -> None:

        # Set up a mock MQTT Device
        publishers = UltrasonicPublishers()
        subscribers = UltrasonicSubscribers()
        topics = Topics(publishers, subscribers)
        self.config = ClientConfig(topics=topics, host='mqtt-broker', port=1883)
        self.device = MqttDevice(client_config=self.config)


    def tearDown(self) -> None:
        self.device.client.loop_stop()
        self.device.client.disconnect()


    def test_client_config_property_types(self):
        """Test for correct property types for the ClientConfig class."""

        self.assertIsInstance(self.config, ClientConfig)
        self.assertIsInstance(self.config.host, str)
        self.assertIsInstance(self.config.port, int)
        self.assertIsInstance(self.config.topics, Topics)
        # Test bad Port
        self.assertRaises(TypeError, ClientConfig, self.config.topics, "mqtt-broker", "1883")


    def test_client_config_property_exists(self):
        """Test to ensure ClientConfig contain properties: client_config, host, port"""

        self.assertTrue(hasattr(self.config, 'topics'), "ClientConfig should have a 'topics' property")
        self.assertTrue(hasattr(self.config, 'host'), "ClientConfig should have a 'host' property")
        self.assertTrue(hasattr(self.config, 'port'), "ClientConfig should have a 'port' property")


    def test_mqtt_device_property_types(self):
        """Test for correct property types for the MqttDevice class."""

        self.assertIsInstance(self.device.host, str)
        self.assertIsInstance(self.device.port, int)
        self.assertIsInstance(self.device.publishers, Publishers)
        self.assertIsInstance(self.device.subscribers, Subscribers)
        self.assertIsInstance(self.device.connected, Connection)

        # Test bad ClientConfig parameter
        self.assertRaises(TypeError, MqttDevice, "client_config")
        # Test default parameters
        self.assertTrue(MqttDevice(ClientConfig(Topics(None, None))))


    def test_mqtt_device_property_exists(self):
        """Test to ensure MqttDevice contain properties: host, port, publishers, subscribers, connected, client"""

        self.assertTrue(hasattr(self.device, 'host'), "ClientConfig should have a 'host' property")
        self.assertTrue(hasattr(self.device, 'port'), "ClientConfig should have a 'port' property")
        self.assertTrue(hasattr(self.device, 'publishers'), "ClientConfig should have a 'publishers' property")
        self.assertTrue(hasattr(self.device, 'subscribers'), "ClientConfig should have a 'subscribers' property")
        self.assertTrue(hasattr(self.device, 'connected'), "ClientConfig should have a 'connected' property")
        self.assertTrue(hasattr(self.device, 'client'), "ClientConfig should have a 'client' property")


    def test_mqtt_device_connect_to_broker(self):
        """Tests the _connect_to_broker method of the MqttDevice class."""

        self.assertTrue(self.device._connect_to_broker())


if __name__ == "__main__":
    unittest.main()
