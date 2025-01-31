import time
import threading
import random
import importlib
from statistics import median
from client.mqtt_client import MqttDevice, ClientConfig, Environment, State


class Ultrasonic(MqttDevice):
    """
    This class is used to collect distances (cm) from the Ultrasonic sensor and publish the distances
    and system data (threads, state, etc..) to the MQTT Broker. All MQTT Broker functionality, such as
    connecting to the broker, publishing, subscribing, and disconnecting is inherited through the MqttDevice
    class. Current distance measurements are sampled at a default rate of ~20Hz and are reported in centimeters.
    The data collected from this sensor is used by the State Machine to inform other services, such as Motor
    related services, to perform tasks like obstacle detection.

    PARAMETERS

    :param    client_config <ClientConfig>:  contains MQTT Broker connection details
    :param                      trig <int>:  the TRIG pin on the Raspberry PI, required for transmitting ultrasound signals
    :param                      echo <int>:  the ECHO pin on the Raspberry PI, required for receiving ultrasound signals
    :param              max_distance <int>:  measurement used to calculate the duration threshold for recording ultrasound signals

    To use this class, start by creating a Topics class using (or updating) instances of the UltrasonicPublishers and UltrasonicSubscribers.
    Create a ClientConfig object using the topics and MQTT Broker connection details, and then create the Ultrasonic device.

    See main.py as an example.

    """

    def __init__(
        self,
        client_config: ClientConfig,
        trig: int = 27,
        echo: int = 22,
    ) -> None:

        if not isinstance(trig, int):
            raise TypeError("Supported types for TRIG GPIO pins are: <int>")
        if not isinstance(echo, int):
            raise TypeError("Supported types for ECHO GPIO pins are: <int>")
        if not isinstance(client_config, ClientConfig):
            raise TypeError("Supported types for client_config are: <ClientConfig>")
        if trig < 1 or echo < 1:
            raise ValueError("GPIO pins cannot be negative.")


        super().__init__(client_config)

        self.trigger_pin = trig
        self.echo_pin = echo
        self.GPIO = None
        self.speed_conversion = 340.0 / 2.0 / 10000.0 # Speed of sound conversion to CM (340 m/s)


        # GPIO Settings
        if self.env == Environment.PROD:
            self._set_GPIO()


    def _set_GPIO(self) -> None:
        """
        Sets the GPIO pins for the Ultrasonic Sensor when the environment is PROD.
        The PROD environment indicates the container is running on the Raspberry Pi
        which is required for importing the RPi.GPIO module.
        """
        try:
            self.GPIO = importlib.import_module("RPi.GPIO")
            # import RPi.GPIO as GPIO
            self.GPIO.setwarnings(False)
            self.GPIO.setmode(self.GPIO.BCM)
            self.GPIO.setup(self.trigger_pin, self.GPIO.OUT)
            self.GPIO.setup(self.echo_pin, self.GPIO.IN)
        except ImportError as e:
            print(f"Error importing library: {e}")
            raise e
        except Exception as e:
            raise e


    def calculate_pulse_time(self) -> float:
        """
        Returns the duration for ultrasound to return to receiver.
        This method should only be called when service is running on
        the Raspberry Pi.
        """

        # Send a 10us pulse to the Trig pin
        self.GPIO.output(self.trigger_pin, self.GPIO.HIGH)
        time.sleep(0.00001)  # 10us
        self.GPIO.output(self.trigger_pin, self.GPIO.LOW)

        # Wait for the Echo pin to go high and start timing
        while self.GPIO.input(self.echo_pin) == self.GPIO.LOW:
            pulse_start = time.time()

        # Wait for the Echo pin to go low and stop timing
        while self.GPIO.input(self.echo_pin) == self.GPIO.HIGH:
            pulse_end = time.time()

        # Calculate the pulse duration
        pulse_duration = (pulse_end - pulse_start) * 1000000
        return pulse_duration


    def calculate_distance(self, samples: int=5) -> float:
        """
        Returns the distance reported by the ultrasonic sensor by applying a median filter on a collection
        of distance samples to remove noise.
        """

        # Return mock data if services are not using hardware
        if self.env == Environment.DEV:
            distance = random.uniform(120.5, 125.7)
            print("Publishing distance: ", distance)
            return distance

        # Generate distance samples - calculate pulse time and convert to distance using SoS conversion
        distance_samples = [self.calculate_pulse_time() * self.speed_conversion for _ in range(samples)]

        return median(distance_samples)


    def run(self, freq: float=0.05):
        """
        Program logic for running the ultrasonic sensor. The sensor continuously publishes distances,
        its current state, and performance metrics such as the active thread count at a specified
        sampling rate.

        :param freq: the sampling rate of the ultrasonic sensor.
        """

        # Connect to MQTT Broker to turn device ON
        self._connect_to_broker()

        while self.state == State.ON:
            time.sleep(freq)
            distance = self.calculate_distance()
            thread_count = threading.active_count()

            responses = [
                self.publish_data(self.publishers.distance, distance, timeout=1.0),
                self.publish_data(self.publishers.threads, thread_count, timeout=1.0),
                self.publish_data(self.publishers.status, self.state, timeout=1.0),
            ]

            if False in responses:
                self.state = State.OFF


    def stop(self):
        """
        This method is used to stop the ultrasonic sensor and is intended to be used to provide functionality
        in addition to the MqttDevice _disconnect() default method. For example, in the future, it could be
        possible to keep the sensor running without publishing data to the MQTT broker.
        """
        self._disconnect()
        if self.env == Environment.PROD:
            self.GPIO.cleanup()
