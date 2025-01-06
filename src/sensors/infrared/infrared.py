import importlib
import threading
from client.mqtt_client import MqttDevice, ClientConfig, Environment, State
from client.topics import Topics, InfraredPublishers, InfraredSubscribers
from enum import IntEnum
from time import sleep



class IRPins(IntEnum):
    """
    Infrared Sensor Pins on vehicle.
    """
    IR01 = 14
    IR02 = 15
    IR03 = 23


class IRLaneDetector(MqttDevice):
    """
    The IRLaneDetector is positioned at the bottom of the vehicle and is used
    to detect when the vehicle crosses a line. It is composed of 3 infrared (IR) sensors
    that emit infrared light and detect the intensity of the light reflected back to
    the sensor, which is particularly useful for detecting lanes on contrasting surfaces
    to support autonomous lane navigation.

    The sensors functionality is encapsulated by the GIOZero LineSensor module and
    primarily report when the sensors detect a dark line through event based callbacks
    such as when_line() and when_no_line().

    PARAMETERS:

    :param client_config:   The MQTT client configuration for subscribing/publishing data
    :param samples:         The amount of samples each sensor collects. Default is 5.
    :param sample_rate:     The number of samples to read per second. Default is 100.
    :param threshold:       The threshold for determining an "active" state based on sample majority. Default is 0.5.
    :param frequency:       The frequency for sampling data from each IR sensor. Default is 20 HZ.
    """

    def __init__(
        self,
        client_config: ClientConfig,
        samples: int = 5,
        sample_rate: int = 100,
        threshold: float = 0.5,
    ) -> None:

        # Brief Type and Value Checks for major issues
        if not isinstance(samples, int):
            raise TypeError("Supported types for samples are: <int>")
        if not isinstance(sample_rate, int):
            raise TypeError("Supported types for sample_rates are: <int>")
        if not isinstance(threshold, float):
            raise TypeError("Supported types for threshold are: <float>")
        if not isinstance(client_config, ClientConfig):
            raise TypeError("Supported types for client_config are: <ClientConfig>")
        if samples < 1 or sample_rate < 1:
            raise ValueError("Samples and sample rates must be positive values.")
        if 1 < threshold < 0:
            raise ValueError("Thresholds to determine active states must be between 0 and 1.")

        # Initialize MqttDevice and connect to broker
        super().__init__(client_config)
        self._connect_to_broker()

        # Import gpiozero if running on Raspberry Pi and set sensors
        gpiozero = self.__import_gpiozero()
        self.IR01 = gpiozero.LineSensor(pin=IRPins.IR01, queue_len=samples, sample_rate=sample_rate, threshold=threshold)
        self.IR02 = gpiozero.LineSensor(pin=IRPins.IR02, queue_len=samples, sample_rate=sample_rate, threshold=threshold)
        self.IR03 = gpiozero.LineSensor(pin=IRPins.IR03, queue_len=samples, sample_rate=sample_rate, threshold=threshold)

    def __import_gpiozero(self) -> None:
        """
        The gpiozero module must be imported on the Raspberry Pi which is determined by
        intentionally setting the system environment to PROD. This protects against
        running the containers on a separate device for testing purposes.
        """
        if self.env != Environment.PROD:
            raise ValueError("The LineDetector container is not running on a Raspberry Pi.")

        try:
            return importlib.import_module("gpiozero")
        except ImportError as e:
            print(f"Error importing library: {e}")
            raise e
        except Exception as e:
            raise e


    def run(self, freq: float=0.05) -> None:
        """
        Program logic for running the IRLaneDetector. The IR sensors actively report
        whether they have detected a dark (black) line and publish to the MQTT broker.

        :param freq: frequency for reading and publishing data.
        """

        while self.state == State.ON:
            sleep(freq)
            print(self.IR01.value, self.IR02.value, self.IR03.value, sep='|')
            thread_count = threading.active_count()

            responses = [
                self.publish_data(self.publishers.irleft, self.IR01.value, timeout=1.0),
                self.publish_data(self.publishers.irmiddle, self.IR02.value, timeout=1.0),
                self.publish_data(self.publishers.irright, self.IR03.value, timeout=1.0),
                self.publish_data(self.publishers.threads, thread_count, timeout=1.0),
                self.publish_data(self.publishers.status, self.state, timeout=1.0),
            ]

            if False in responses:
                self.stop()

    def stop(self) -> None:
        """
        Set the state to OFF and publish the status.
        """
        self.state = State.OFF
        self.publish_data(self.publishers.status, self.state, timeout=1.0)



if __name__ == '__main__':

    # Set the topics for MQTT
    publishers = InfraredPublishers()
    subscribers = InfraredSubscribers()
    topics = Topics(publishers, subscribers)
    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)

    sensor = IRLaneDetector(client_config=clientConfig)

    try:
        sensor.run()
    except KeyboardInterrupt:
        print("LaneDetector stopped...")
        sensor.stop()

