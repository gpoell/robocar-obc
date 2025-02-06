import paho.mqtt.client as mqtt
import os
from enum import Enum, IntEnum
from client.topics import Topics, Topic, get_topics
# from topics import Topics, Topic, get_topics      # Uncomment for testing in containers

class Environment(Enum):
    DEV = 0
    PROD = 1

class State(IntEnum):
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
        self.state = State.OFF
        self.client = mqtt.Client()
        self.env = self._set_environment()
        self.client.on_connect = self._client_on_connect
        self.client.on_disconnect = self._client_on_disconnect
        self._connect_to_broker()


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


    def _client_on_connect(self, client, userdata, flags, return_code) -> bool:
        """Callback for when device connects to MQTT broker."""
        if return_code != 0:
            print("Device could not connect, return code:", return_code)
            return False

        print("Device connected...")
        return True


    def _client_on_disconnect(self, client, userdata, return_code) -> bool:
        """Callback for when device connects to MQTT broker."""
        if return_code != 0:
            print("Device faced an error on disconnect, return code:", return_code)
            return False

        print("Device successfully disconnected...")
        return True


    def _connect_to_broker(self) -> None:    # Future state will raise exceptions with custom class
        """
        This method encapsulates functionality for connecting to the MQTT broker, subsribing to topics,
        and starting the client event loop, returning a MQTTErrorCode that is used to set the connection status of the device.

        MQTTErrorCode values range from [-1, 16] with 0 representing "No Error" or "Success"
        """

        try:
            # Connect to the broker, subscribe to topics, and start event loop
            self.client.connect(self.host, self.port)
            subscriptions = get_topics(self.subscribers)
            self.client.subscribe(subscriptions)
            self.client.loop_start()

        except Exception as e:
            # Disconnect on any error and raise built in exception (future state fix ambiguity)
            self.disconnect()
            raise e

        # Turn device on
        self.state = State.ON


    def _disconnect(self) -> None:
        """
        Method to disconnect the client from the MQTT broker. Ensures the state is set to OFF
        and is published to the broker to enform the state machine, and disconnects the client.
        """
        self.state = State.OFF
        if self.client.is_connected():
                self.publish_data(self.publishers.status, self.state, timeout=1.0),
                self.client.loop_stop()
                self.client.disconnect()


    def _set_environment(self) -> Environment:
        """
        Method to set the environment state for each device to drive environment specific behavior.
        For example, the Ultrasonic sensor requires the GPIO library which can only be imported
        when the container is running on the physical hardware.
        """
        if os.environ["ENVIRON"] == "PROD":
            return Environment.PROD
        return Environment.DEV


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
