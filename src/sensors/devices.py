import paho.mqtt.client as mqtt
import asyncio
from enum import Enum
from dataclasses import dataclass
from typing import Union

class Topic:
    """This type is intended to be used for all MQTT Topics, ensuring correct properties are defined for publishing and subscribing data."""
    
    def __init__(
        self,
        topic: str,
        qos: int
    ) -> None:

        assert type(topic) == str, "Supported types of topics are: str"
        assert type(qos) == int, "Supported types of qos are: int"
        assert qos < 3 and qos >= 0, "MQTT clients only support QOS levels 0-3"

        self.topic = topic
        self.qos = qos

    
    def get_values(self) -> tuple:
        """Returns topic values. Helpful for subscribing and publishing to multiple topics in MqttDevice class."""
        vals = [val for attr, val in self.__dict__.items()]
        return tuple(vals)

@dataclass
class Publishers:
    """Defines properties for topics intended for publishing data. These values vary based on device."""

    def get_topics(self) -> list[tuple]:
        """Returns a list of topics for each property."""
        topics = [val for _, val in self.__dict__.items()]
        return [topic.get_values() for topic in topics]

@dataclass
class Subscribers:
    """Defines properties for topics intended for subscribing to data. These values vary based on device."""
    
    def get_topics(self) -> list[tuple]:
        """Returns a list of topics for each property."""
        topics = [val for _, val in self.__dict__.items()]
        return [topic.get_values() for topic in topics]


@dataclass
class Topics:
    """
    A collection of topics intended for publishing and subscribing to the MQTT broker.

    :param publishers: defines <Publishing> topics for the respective device.
    :param subscribers: defines <Subscriber> topics for the respective device. 
    """
    def __init__(
        self,
        publishers: Union[Publishers, None],
        subscribers: Union[Subscribers, None]
    ) -> None:

        assert type(publishers) == Publishers or type(publishers) == None, "Supported types of publishers are: Publishers, None"
        assert type(subscribers) == Subscribers or type(subscribers) == None, "Supported types of subscribers are: Subscribers, None"

        self.publishers = publishers
        self.subscribers = subscribers

    def get_publishers(self) -> list[tuple]:
        if self.publishers == None:
            return [()]
        return self.publishers.get_topics()

    def get_subscribers(self) -> list[tuple]:
        if self.subscribers == None:
            return [()]
        return self.subscribers.get_topics()


class ClientConfig:
    """
    Data type required for connecting MQTT Devices to a MQTT broker, helping enforce correct
    value types and ranges. This should be defined at the beginning of a program and used to
    instantiate the respective Device (i.e. ultrasonic sensor, motor, camera, etc..)
    """

    def __init__(
        self,
        topics: Topics,
        host: str = "mqtt-broker",
        port: int = 1883
    ) -> None:

        assert type(host) == str, "Supported host types are: str"
        assert type(port) == int, "Supported port types are: int"
        assert type(topics) == Topics, "Supported topics types are: Topics"
        
        self.host: str = host
        self.port: int = port
        self.topics: Topics = topics
    


class MqttDevice:
    """MQTT Devices are intended to be used as base classes for sensors and devices attached to the robocar.
       These devices are composed of MQTT clients that are required for publishing and subscribing to data
       that inform the overall system's behavior; including the device status and performance metrics.
       These Devices require the following attributes:

       :param clientConfig: ClientConfig type providing details for MQTT host, port, and topics. This
            specific type provides necessary restrictions to align values with the MQTT client.


       Important methods listed below provide high level functionality for connecting to MQTT brokers, error
       checking, and managing device states.

        - connect_to_broker() -> bool: connects the device to the MQTT broker, subscribes to topics, and starts
            the client loop to actively listen for incoming messages.

        - subscribe(): wrapper function for the client subscribe method with built in exception handling.

        - check_published_msg(mqttMessage, timeout) -> bool: verifies if the message is published after a 
            a defined duration.
    """

    def __init__(
        self,
        clientConfig: ClientConfig
    ) -> None:

        assert type(clientConfig) == ClientConfig, "Supported types of clientConfig are: ClientConfig"

        self.host = clientConfig.host
        self.port = clientConfig.port
        self.publishers = clientConfig.topics.publishers
        self.subscribers = clientConfig.topics.subscribers
        self.connected = False
        self.client = mqtt.Client()


    def _connect_to_broker(self) -> bool:    # Future state will raise exceptions with custom class
        """
        This method encapsulates functionality for connecting to the MQTT broker, subsribing to topics,
        and starting the client event loop, returning a MQTTErrorCode that is used to set the connection status of the device.

        MQTTErrorCode values range from [-1, 16] with 0 representing "No Error" or "Success"

        Example: self.connected = if not connect_to_broker() (if there is no error, set to True)
        """
        # Connect to MQTT Client
        connection_err = self.client.connect(self.host, self.port)
        if connection_err:
            return False
        
        # Subscribe to topics
        subscriptions = self.subscribers.get_topics()
        self.client.subscribe(subscriptions)

        # Start Client
        loop_error = self.client.loop_start()
        if loop_error:
            return False

        # return mqtt.MQTTErrorCode.MQTT_ERR_SUCCESS
        return True

    
    def publish_data(self, topic: Topic, message, timeout=1.0):
        """
        Publishes data to the MQTT broker and verifies the connection status.
        
        :param topic: Topic to publish
        :param message: data to publish
        :param timeout: the length of time to wait for a successful response
        """

        mqttMessage = self.client.publish(
            topic = topic.topic,
            qos = topic.qos,
            payload = message
        )
        return self._validate_msg_publish(mqttMessage, topic, timeout)


    def _validate_msg_publish(self, mqttMessage, topic, timeout=1.0) -> bool:
        """
        Validates if the message was successfully published to the broker and will wait for a specified duration
        
        :param mqttMessage: a MQTTMessage response object for the published message
        :param timeout: a duration (seconds) to wait for the message to publish to the broker.
        """
        
        mqttMessage.wait_for_publish(timeout=timeout)

        if not mqttMessage.is_published():
            print("Failed to send message to topic " + topic)
            # Future state will incorporate retries or a failure counter to disconnect

            if not self.client.is_connected():
                print("Client not connected, exiting...")
                return False

        return True


