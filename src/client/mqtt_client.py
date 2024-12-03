import paho.mqtt.client as mqtt
from enum import IntEnum
from topics import Topics, Topic, get_topics

class Connection(IntEnum):
    OFF = 0
    ON = 1

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

        if not isinstance(topics, Topics):
            raise TypeError("Supported types for topics are: <Topics>")

        if not isinstance(host, str):
            raise TypeError("Supported types for hosts are: <str>")

        if not isinstance(port, int):
            raise TypeError("Supported types for ports are: <int>")

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
        client_config: ClientConfig
    ) -> None:

        if not isinstance(client_config, ClientConfig):
            raise TypeError("Supported types for topic are: <ClientConfig>")


        self.host = client_config.host
        self.port = client_config.port
        self.publishers = client_config.topics.publishers
        self.subscribers = client_config.topics.subscribers
        self.connected = Connection.OFF
        self.client = mqtt.Client()


    def _connect_to_broker(self) -> bool:    # Future state will raise exceptions with custom class
        """
        This method encapsulates functionality for connecting to the MQTT broker, subsribing to topics,
        and starting the client event loop, returning a MQTTErrorCode that is used to set the connection status of the device.

        MQTTErrorCode values range from [-1, 16] with 0 representing "No Error" or "Success"

        Example: self.connected = if not connect_to_broker() (if there is no error, set to True)
        """

        try:
            self.client.connect(self.host, self.port)
            subscriptions = get_topics(self.subscribers)
            self.client.subscribe(subscriptions)
            self.client.loop_start()

        except Exception as e:
            self.disconnect()
            raise e     # Raise the default/built in exceptions with Paho MQTT

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


    def disconnect(self):
        """Method to disconnect the client from the MQTT broker"""
        if self.client.is_connected():
                self.client.loop_stop()
                self.client.disconnect()