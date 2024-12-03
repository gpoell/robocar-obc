import time
import threading
import random
import importlib
# import RPi.GPIO as GPIO
# from devices import MqttDevice, ClientConfig
from client.mqtt_client import MqttDevice, ClientConfig, Environment, State


class Ultrasonic(MqttDevice):

    def __init__(
        self,
        clientConfig: ClientConfig,
        trig: int = 27,
        echo: int = 22,
        maxDistance: int = 30
    ) -> None:

        assert type(maxDistance) == int, "Supported types for maxDistance are: int"
        assert maxDistance > 0 and maxDistance < 1000, "The maximum distance should be positive and less than 1000cm."
        assert type(trig) == int and trig > 0, "GPIO pins must be integers and cannot be negative."
        assert type(echo) == int and trig > 0, "GPIO pins must be integers and cannot be negative."
        assert type(clientConfig) == ClientConfig, "Supported types for clientConfig are: ClientConfig"

        super().__init__(clientConfig)

        self.trigger_pin = trig
        self.echo_pin = echo
        self.MAX_DISTANCE = maxDistance         # define the maximum measuring distance, unit: cm
        self.timeOut = self.MAX_DISTANCE * 60   # calculate timeout according to the maximum measuring distance
        # self.client = mqtt.Client()             # Each sensor/module should manage its own client
        self.GPIO = None

        # GPIO Settings
        if self.env == Environment.PROD:
            self._set_GPIO()

        # Attach MQTT Event-Based Callbacks to Client
        self.client.on_connect = self.client_on_connect
        self.client.on_message = self.client_on_message


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


    def pulseIn(self, pin, level, timeOut):  # obtain pulse time of a pin under timeOut
        t0 = time.time()
        while (self.GPIO.input(pin) != level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0

        t0 = time.time()
        while (self.GPIO.input(pin) == level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0

        pulseTime = (time.time() - t0) * 1000000
        return pulseTime


    def get_distance(self, samples: int=5) -> float:  # get the measurement results of ultrasonic module,with unit: cm
        """
        Returns the distance reported by the ultrasonic sensor by applying a median filter on a collection
        of distance samples to remove noise.
        """

        # Return mock data if services are not using hardware
        if self.env == Environment.DEV:
            distance = random.uniform(120.5, 125.7)
            print("Publishing distance: ", distance)
            return distance

        # List comprehension here on new method
        distance_samples = [0, 0, 0, 0, 0]

        # Abstract this into two functions- 1 should calculate ping time2 should calculate distance
        for i in range(samples):
            self.GPIO.output(self.trigger_pin, self.GPIO.HIGH)  # make trigger_pin output 10us HIGH level
            time.sleep(0.00001)  # 10us
            self.GPIO.output(self.trigger_pin, self.GPIO.LOW)  # make trigger_pin output LOW level
            pingTime = self.pulseIn(self.echo_pin, self.GPIO.HIGH, self.timeOut)  # read plus time of echo_pin
            distance_samples[i] = pingTime * 340.0 / 2.0 / 10000.0  # calculate distance with sound speed 340m/s
        distance_samples = sorted(distance_samples)
        return distance_samples[2]


    def client_on_connect(self, client, userdata, flags, return_code) -> bool:
        """Client callback when ultrasonic sensor connects to MQTT broker. Temporary placehoder"""
        if return_code != 0:
            print("could not connect, return code:", return_code)
            return False

        print("Ultrasonic sensor is connected.....")
        return True


    def client_on_message(self, client, userdata, msg):
        """
        Event based callback when the ultrasonic sensor receives a message from MQTT broker.
        The types of messages received by the broker should reflect what is declared in the
        Subscribers datatype,

        :param client: reference to client instance connected to MQTT broker.
        :param userdata: <TBD>
        :param msg: incoming message from MQTT Broker.
        """
        message = msg.payload.decode()
        topic = msg.topic

        if topic == self.subscribers.appStatus and message != "active":
            self.state = State.OFF


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
            distance = self.get_distance()
            thread_count = threading.active_count()

            responses = [
                self.publish_data(self.publishers.distance, distance, timeout=1.0),
                self.publish_data(self.publishers.threads, thread_count, timeout=1.0),
                self.publish_data(self.publishers.status, self.state, timeout=1.0),
            ]

            self.client.publish("device/motor/threads", 8)        # TEMPORARY FOR VISUALIZATION

            if False in responses:
                self.state = State.OFF

    def stop(self):
        """
        This method is used to stop the ultrasonic sensor and is intended to be used to provide functionality
        in addition to the MqttDevice _disconnect() default method. For example, in the future, it could be
        possible to keep the sensor running without publishing data to the MQTT broker.
        """
        self._disconnect()
