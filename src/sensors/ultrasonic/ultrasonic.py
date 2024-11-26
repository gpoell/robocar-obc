import time
import threading
import asyncio
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from enum import IntEnum
from devices import MqttDevice, ClientConfig

class State(IntEnum):
    OFF = 0
    ON = 1


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

        self.state = State.OFF
        self.trigger_pin = trig
        self.echo_pin = echo
        self.MAX_DISTANCE = maxDistance         # define the maximum measuring distance, unit: cm
        self.timeOut = self.MAX_DISTANCE * 60   # calculate timeout according to the maximum measuring distance
        self.client = mqtt.Client()             # Each sensor/module should manage its own client

        # GPIO Settings
        self._set_GPIO()

        # Attach MQTT Event-Based Callbacks to Client
        self.client.on_connect = self.client_on_connect
        self.client.on_message = self.client_on_message


    def _set_GPIO(self):
        """Sets the GPIO pins for the Ultrasonic Sensor."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    
    def pulseIn(self, pin, level, timeOut):  # obtain pulse time of a pin under timeOut
        t0 = time.time()
        while (GPIO.input(pin) != level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0
        
        t0 = time.time()
        while (GPIO.input(pin) == level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0
        
        pulseTime = (time.time() - t0) * 1000000
        return pulseTime


    def get_distance(self) -> float:  # get the measurement results of ultrasonic module,with unit: cm
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(5):
            GPIO.output(self.trigger_pin, GPIO.HIGH)  # make trigger_pin output 10us HIGH level
            time.sleep(0.00001)  # 10us
            GPIO.output(self.trigger_pin, GPIO.LOW)  # make trigger_pin output LOW level
            pingTime = self.pulseIn(self.echo_pin, GPIO.HIGH, self.timeOut)  # read plus time of echo_pin
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0  # calculate distance with sound speed 340m/s
        distance_cm = sorted(distance_cm)
        return distance_cm[2]


    def client_on_connect(self, client, userdata, flags, return_code):
        if return_code == 0:
            print("Ultrasonic sensor is connected.....")
        else:
            print("could not connect, return code:", return_code)


    def client_on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        topic = msg.topic
        
        if topic == self.subscribers.appStatus and message != "active":
            self.state = State.OFF


    def run(self, freq: float=0.05):
        self.status = self._connect_to_broker()
        while self.status:
            time.sleep(freq)
            distance = self.get_distance()
            threadCount = threading.active_count()

            start_time = time.perf_counter()
            
            responses = [
                self.publish_data(self.publishers.distance, distance, timeout=1.0),
                self.publish_data(self.publishers.threads, threadCount, timeout=1.0),
                self.publish_data(self.publishers.status, self.status, timeout=1.0),
            ]

            self.client.publish("device/motor/threads", 8)        # TEMPORARY FOR VISUALIZATION

            if False in responses:
                self.status = False

    def stop(self):
        print("Ultrasonic sensor has been stopped...")
        self.status = False
        self.client.publish(
            topic = self.publishers.status.topic,
            qos = self.publishers.status.qos,
            payload = State.OFF
        )
        self.client.loop_stop()
        self.client.disconnect()