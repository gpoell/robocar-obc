from time import sleep
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
        self.client.on_message = self.client_on_message


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
        self.motor.drive(
            l_u_duty=lu_pwm,
            l_l_duty=ll_pwm,
            r_u_duty=ru_pwm,
            r_l_duty=rl_pwm
        )

    def stop(self) -> None:
        """Stops the motor and disconnect from the MQTT Broker."""
        self.motor.stop()
        self._disconnect()


    def drive_left(self, left_wheels_duty: int, right_wheels_duty: int) -> None:
        """
        Provide positive PWM duty cycles to drive the vehicle to the left by supplying higher power to the right
        wheels and lower power to the left wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        left_wheels_duty = abs(left_wheels_duty)
        right_wheels_duty = abs(right_wheels_duty)

        if right_wheels_duty < left_wheels_duty:
            raise ValueError("Driving to the left requires higher PWM duty cycles for the right wheels.")

        self.motor.drive(left_wheels_duty, left_wheels_duty, right_wheels_duty, right_wheels_duty)


    def drive_right(self, left_wheels_duty: int, right_wheels_duty: int) -> None:
        """
        Provide positive PWM duty cycles to drive the vehicle to the right by supplying higher power to the right
        wheels and lower power to the left wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        left_wheels_duty = abs(left_wheels_duty)
        right_wheels_duty = abs(right_wheels_duty)

        if right_wheels_duty > left_wheels_duty:
            raise ValueError("Driving to the right requires higher PWM duty cycles for the left wheels.")

        self.motor.drive(left_wheels_duty, left_wheels_duty, right_wheels_duty, right_wheels_duty)


    def reverse(self, pwm_duty: int) -> None:
        """
        Provide a positive PWM duty cycle to reverse the vehicle
        by supplying consistent power to all 4 wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        pwm_duty = abs(pwm_duty)

        self.motor.drive(-pwm_duty, -pwm_duty, -pwm_duty, -pwm_duty)


    def reverse_left(self, left_wheels_duty: int, right_wheels_duty: int) -> None:
        """
        Provide positive PWM duty cycles to reverse the vehicle to the left by supplying higher power to the right
        wheels and lower power to the left wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        left_wheels_duty = abs(left_wheels_duty)
        right_wheels_duty = abs(right_wheels_duty)

        if right_wheels_duty < left_wheels_duty:
            raise ValueError("Reversing to the left requires higher PWM duty cycles for the right wheels.")

        self.motor.drive(-left_wheels_duty, -left_wheels_duty, -right_wheels_duty, -right_wheels_duty)


    def reverse_right(self, left_wheels_duty: int, right_wheels_duty: int) -> None:
        """
        Provide positive PWM duty cycles to reverse the vehicle to the right by supplying higher power to the left
        wheels and lower power to the right wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        left_wheels_duty = abs(left_wheels_duty)
        right_wheels_duty = abs(right_wheels_duty)

        if right_wheels_duty > left_wheels_duty:
            raise ValueError("Reversing to the right requires higher PWM duty cycles for the left wheels.")

        self.motor.drive(-left_wheels_duty, -left_wheels_duty, -right_wheels_duty, -right_wheels_duty)


    def turn_around(self, turn_cycles: int=3) -> None:
        """
        (Testing) Turns the vehicle approximately 180 degrees.
        Multiple variables such as floor friction and differential speed ratios
        impact the turning radius.

        This method requires calibration. Currently, 4 turn cycle results in a 90
        degree turn, so 8 cycles is 180 degrees.
        """

        for _ in range(turn_cycles):
            self.drive_left(300, 3200)
            sleep(0.5)

            # Pause
            self.set_motor_model(0, 0, 0, 0)
            sleep(0.5)

            self.reverse_right(3200, 300)
            sleep(0.5)

            # Pause
            self.motor.drive(0, 0, 0, 0)
            sleep(0.5)


        self.motor.drive(0, 0, 0, 0)
