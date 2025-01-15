from client.mqtt_client import MqttDevice, ClientConfig, State
from motor.motor import Motor

class Vehicle(MqttDevice):
    """
    The Vehicle is the primary component used to drive the car. It is
    comprised of a motor (see motor.py) that determines the speed
    and direction of the vehicle, and inherits from MqttDevice
    to actively listen to data published by the Ultrasonic and Lane
    Detector sensors.

    Vehicle behavior is determined through pipelines that solve specific
    challenges such as detecting obstacles and lane navigation.
    """

    def __init__(
        self,
        client_config: ClientConfig,
    ) -> None:

        if not isinstance(client_config, ClientConfig):
            raise TypeError("Supported types for client_config are: <ClientConfig>")

        super().__init__(client_config)

        self.motor = Motor()
        self.target_distance = 0      # Track distances from ultrasonic sensor
        self.lane_model = [0, 0, 0]   # Track status of IR lane detectors

        # Attach MQTT Event-Based Callbacks to Client
        self.client.on_connect = self.client_on_connect
        self.client.on_message = self.client_on_message
        self._connect_to_broker()


    def client_on_connect(self, client, userdata, flags, return_code) -> None:
        """Callback when the device connects to the MQTT Broker."""
        if return_code != 0:
            raise ValueError("Could not connect to MQTT Broker, return code:", return_code)

        print("Vehicle is connected to the MQTT Broker...")


    def client_on_message(self, client, userdata, msg) -> None:
        """Callback when device receives a message from the MQTT Broker."""
        message = msg.payload.decode()
        topic = msg.topic

        if topic == self.subscribers.appStatus and message != "active":
            self.state = State.OFF

        if topic == self.subscribers.ultrasonicDistance.topic:
            self.target_distance = float(message)

        if topic == self.subscribers.irleft.topic:
            self.lane_model[0] = int(message)

        if topic == self.subscribers.irmiddle.topic:
            self.lane_model[1] = int(message)

        if topic == self.subscribers.irright.topic:
            self.lane_model[2] = int(message)

    def drive(self, lu_pwm: int, ll_pwm: int, ru_pwm: int, rl_pwm: int) -> None:
        """
        Sets the speed and direction of the vehicle by providing PWM duty cycles
        for all of the wheels.
        """
        self.motor.set_motor_model(
            left_upper_duty=lu_pwm,
            left_lower_duty=ll_pwm,
            right_upper_duty=ru_pwm,
            right_lower_duty=rl_pwm
        )

    def stop(self) -> None:
        """Stops the motor and disconnect from the MQTT Broker."""
        self.motor.stop()
        self._disconnect()