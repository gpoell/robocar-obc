import unittest
from client.topics import Topics, Topic, Publishers, Subscribers, \
    UltrasonicPublishers, UltrasonicSubscribers, \
    VehiclePublishers, VehicleSubscribers,  \
    InfraredPublishers, InfraredSubscribers, \
    get_topics


class TestTopics(unittest.TestCase):
    """Test class for Topic related data types."""

    def setUp(self):
        ultra_pubs = UltrasonicPublishers()
        ultra_subs = UltrasonicSubscribers()
        self.topics = Topics(publishers=ultra_pubs, subscribers=ultra_subs)


    def test_topic_property_types(self):
        """
        Tests correct property types for the Topic class.
        Future state should leverage hypothesis module for more variety.
        """

        # Arrange
        good_topic = "/device/sensor/data"
        good_qos = 1
        bad_topic_type = 54
        bad_qos_type = 1.0
        bad_qos_value = -5

        # Assert and Act
        self.assertRaises(TypeError, Topic, bad_topic_type, good_qos)
        self.assertRaises(TypeError, Topic, good_topic, bad_qos_type)
        self.assertRaises(ValueError, Topic, good_topic, bad_qos_value)


    def test_topic_properties_exist(self):
        """Test to ensure Topic contains properties: topic, qos"""

        topic = Topic("/topic/data", 1)
        self.assertTrue(hasattr(topic, 'topic'), "Topic class should have a 'topic' property")
        self.assertTrue(hasattr(topic, 'qos'), "Topic class should have a 'qos' property")


    def test_topic_get_values(self):
        """Test Topic method for returning topic values as a tuple."""

        topic = "device/ultrasonic/distance"
        qos = 0
        topic_example = Topic(topic=topic, qos=qos)

        self.assertTupleEqual(topic_example.get_values(), (topic, qos))


    def test_topics_properties_types(self):
        """Test for correct property types for the Topics class."""
        topics2 = Topics(None, None)

        self.assertIsInstance(self.topics.publishers, Publishers)
        self.assertIsInstance(self.topics.subscribers, Subscribers)
        self.assertIs(topics2.publishers, None)
        self.assertIs(topics2.subscribers, None)
        self.assertRaises(AssertionError, Topics, "/some/value", 400)


    def test_topics_properties_exists(self):
        """Test to ensure Topics contain properties: topic, qos"""

        self.assertTrue(hasattr(self.topics, 'publishers'), "Topics class should have a 'publishers' property")
        self.assertTrue(hasattr(self.topics, 'subscribers'), "Topics class should have a 'subscribers' property")


    def test_topics_subs_instance(self):
        """Tests to ensure Topic publishers are Publisher type"""
        self.assertIsInstance(self.topics.publishers, Publishers)


    def test_ultrasonicpublishers_properties_exist(self):
        """Test to ensure UltrasonicPublisher instances contain properties: distance, threads, status"""

        ultra_pub = UltrasonicPublishers()
        self.assertTrue(hasattr(ultra_pub, 'distance'), "UltrasonicPublishers should have a 'distance' property")
        self.assertTrue(hasattr(ultra_pub, 'threads'), "UltrasonicPublishers should have a 'threads' property")
        self.assertTrue(hasattr(ultra_pub, 'status'), "UltrasonicPublishers should have a 'status' property")


    def test_ultrasonicpublishers_property_types(self):
        """Test to ensure UltrasonicPublisher instances contain Topic values."""
        ultra_pub = UltrasonicPublishers()

        self.assertIsInstance(ultra_pub.distance, Topic)
        self.assertIsInstance(ultra_pub.threads, Topic)
        self.assertIsInstance(ultra_pub.status, Topic)


    def test_ultrasonicsubscribers_properties_exist(self):
        """Test to ensure UltrasonicSubscribers contain properties: appStatus"""

        ultra_sub = UltrasonicSubscribers()
        self.assertTrue(hasattr(ultra_sub, 'appStatus'), "UltrasonicSubscribers should have a 'appStatus' property")


    def test_ultrasonicsubscribers_property_types(self):
        """Test to ensure UltrasonicSubscribers contain Topic values."""
        ultra_sub = UltrasonicSubscribers()

        self.assertIsInstance(ultra_sub.appStatus, Topic)


    def test_motor_publishers_properties_exist(self):
        """
        Test to ensure MotorPublisher instances contain properties:
        - luPWM
        - llPWM
        - ruPWM
        - rlPWM
        - threads
        - status
        """

        motor_pub = VehiclePublishers()

        self.assertTrue(hasattr(motor_pub, 'luPWM'), "VehiclePublishers should have a 'luPWM' property")
        self.assertTrue(hasattr(motor_pub, 'llPWM'), "VehiclePublishers should have a 'llPWM' property")
        self.assertTrue(hasattr(motor_pub, 'ruPWM'), "VehiclePublishers should have a 'ruPWM' property")
        self.assertTrue(hasattr(motor_pub, 'rlPWM'), "VehiclePublishers should have a 'rlPWM' property")
        self.assertTrue(hasattr(motor_pub, 'threads'), "VehiclePublishers should have a 'threads' property")
        self.assertTrue(hasattr(motor_pub, 'status'), "VehiclePublishers should have a 'status' property")


    def test_motor_publishers_property_types(self):
        """Test to ensure MotorPublisher instances contain Topic values."""

        motor_pub = VehiclePublishers()

        self.assertIsInstance(motor_pub.luPWM, Topic)
        self.assertIsInstance(motor_pub.llPWM, Topic)
        self.assertIsInstance(motor_pub.ruPWM, Topic)
        self.assertIsInstance(motor_pub.rlPWM, Topic)
        self.assertIsInstance(motor_pub.threads, Topic)
        self.assertIsInstance(motor_pub.status, Topic)


    def test_motor_subscribers_properties_exist(self):
        """Test to ensure VehicleSubscribers contain properties: appStatus"""

        motor_sub = VehicleSubscribers()

        self.assertTrue(hasattr(motor_sub, 'appStatus'), "VehicleSubscribers should have a 'appStatus' property")
        self.assertTrue(hasattr(motor_sub, 'ultrasonicDistance'), "VehicleSubscribers should have a 'ultrasonicDistance' property")
        self.assertTrue(hasattr(motor_sub, 'irleft'), "VehicleSubscribers should have a 'irleft' property")
        self.assertTrue(hasattr(motor_sub, 'irmiddle'), "VehicleSubscribers should have a 'irmiddle' property")
        self.assertTrue(hasattr(motor_sub, 'irright'), "VehicleSubscribers should have a 'irright' property")


    def test_motor_subscribers_property_types(self):
        """Test to ensure VehicleSubscribers contain Topic values."""

        motor_sub = VehicleSubscribers()

        self.assertIsInstance(motor_sub.appStatus, Topic)


    def test_infrared_publishers_properties_exist(self):
        """
        Test to ensure InfraredPublishers instances contain properties:
        - irleft
        - irmiddle
        - irright
        - threads
        - status
        """

        publishers = InfraredPublishers()

        self.assertTrue(hasattr(publishers, 'irleft'), "InfraredPublishers should have a 'irleft' property")
        self.assertTrue(hasattr(publishers, 'irright'), "InfraredPublishers should have a 'irright' property")
        self.assertTrue(hasattr(publishers, 'irmiddle'), "InfraredPublishers should have a 'ruPWM' property")
        self.assertTrue(hasattr(publishers, 'threads'), "InfraredPublishers should have a 'threads' property")
        self.assertTrue(hasattr(publishers, 'status'), "InfraredPublishers should have a 'status' property")


    def test_infrared_publishers_property_types(self):
        """Test to ensure InfraredPublisher instances contain Topic values."""

        publishers = InfraredPublishers()

        self.assertIsInstance(publishers.irleft, Topic)
        self.assertIsInstance(publishers.irmiddle, Topic)
        self.assertIsInstance(publishers.irright, Topic)
        self.assertIsInstance(publishers.threads, Topic)
        self.assertIsInstance(publishers.status, Topic)


    def test_get_topics(self):
        """
        Test for the get_topics method that returns a list of tuples containing topic and qos values
        for publishers and subscribers.
        """
        bad_mock_topics = [
            Topic("device/ultrasonic/data", 0),
            Topic("device/ultrasonic/data2", 0),
        ]

        successful_response_pub = [('device/ultrasonic/distance', 0), ('device/ultrasonic/status', 0), ('device/ultrasonic/threads', 0)]
        successful_response_sub = [('app/status', 1)]

        self.assertRaises(TypeError, get_topics, bad_mock_topics)
        self.assertEqual(successful_response_pub, get_topics(self.topics.publishers))
        self.assertEqual(successful_response_sub, get_topics(self.topics.subscribers))
        self.assertIsInstance(get_topics(self.topics.publishers), list)
        self.assertIsInstance(get_topics(self.topics.publishers)[0], tuple)


if __name__ == "__main__":
    unittest.main()
