from time import sleep
from client.mqtt_client import MqttDevice, ClientConfig, State
from parts.motor import Motor

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value

        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        """Calculate PID output given the current system value and time step."""
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error

        return output


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
        self.lane_error = 0           # Track CV lane detection error

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

        if topic == self.subscribers.cvLaneError.topic:
            self.lane_error = int(message)

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

    def speed(self) -> list[int]:
        """
        Returns the "speed" of the vehicle which is a representation of each
        wheels PWM duty cycle in the order:
            1. Left Upper
            2. Left Lower
            3. Right Upper
            4. Right Lower
        """
        return [
            self.motor.wheel_left_upper.speed,
            self.motor.wheel_left_lower.speed,
            self.motor.wheel_right_upper.speed,
            self.motor.wheel_right_lower.speed
        ]


if __name__ == "__main__":
    from client.topics import VehiclePublishers, VehicleSubscribers, Topics
    from client.mqtt_client import ClientConfig
    from time import sleep

    # Create Motor with topics and MQTT connection details
    publishers = VehiclePublishers()
    subscribers = VehicleSubscribers()
    topics = Topics(publishers, subscribers)
    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)
    vehicle = Vehicle(clientConfig)

    try:
        vehicle.drive(2000, 2000, 2000, 2000)  # Forward
        sleep(1)
        vehicle.stop()
    except KeyboardInterrupt:
        vehicle.stop()