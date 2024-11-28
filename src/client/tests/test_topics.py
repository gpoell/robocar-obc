import unittest
from topics import Topics, Topic, UltrasonicPublishers, UltrasonicSubscribers, Publishers, Subscribers

class TestTopics(unittest.TestCase):
    """Test class for Topic related data types."""

    def setUp(self):
        ultra_pubs = UltrasonicPublishers()
        ultra_subs = UltrasonicSubscribers()
        self.topic = Topics(publishers=ultra_pubs, subscribers=ultra_subs)


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

        self.assertIsInstance(self.topic.publishers, Publishers)
        self.assertIsInstance(self.topic.subscribers, Subscribers)
        self.assertIs(topics2.publishers, None)
        self.assertIs(topics2.subscribers, None)
        self.assertRaises(AssertionError, Topics, "/some/value", 400)

    def test_topics_properties_exists(self):
        """Test to ensure Topics contain properties: topic, qos"""

        self.assertTrue(hasattr(self.topic, 'publishers'), "Topics class should have a 'publishers' property")
        self.assertTrue(hasattr(self.topic, 'subscribers'), "Topics class should have a 'subscribers' property")


    def test_topics_subs_instance(self):
        """Tests to ensure Topic publishers are Publisher type"""
        self.assertIsInstance(self.topic.publishers, Publishers)

    # test get_topics()

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
        """Test to ensure UltrasonicSubscribers contain properties: distance, threads, status"""

        ultra_sub = UltrasonicSubscribers()
        self.assertTrue(hasattr(ultra_sub, 'appStatus'), "UltrasonicSubscribers should have a 'distance' property")


    def test_ultrasonicsubscribers_property_types(self):
        """Test to ensure UltrasonicSubscribers contain Topic values."""
        ultra_sub = UltrasonicSubscribers()

        self.assertIsInstance(ultra_sub.appStatus, Topic)


if __name__ == "__main__":
    unittest.main()
