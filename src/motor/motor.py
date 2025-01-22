import time
from dataclasses import dataclass
from PCA9685 import PCA9685


@dataclass(frozen=True)
class MotorTerminals:
    """Motor terminals for each wheel."""

    left_upper_forward: int = 0
    left_upper_reverse: int = 1
    left_lower_reverse: int = 2
    left_lower_forward: int = 3
    right_lower_forward: int = 4
    right_lower_reverse: int = 5
    right_upper_forward: int = 6
    right_upper_reverse: int = 7


class Wheel:
    """
    Wheels are powered by DC motors that supply power to terminals
    to determine the speed and direction which they turn.

    @param  forward:    datasheet specification for forward terminal
    @param  reverse:    datasheet specification for reverse terminal
    """

    def __init__(
        self,
        forward: MotorTerminals,
        reverse: MotorTerminals
    ) -> None:

        if not isinstance(forward, (MotorTerminals)):
            raise TypeError("Supported types for wheel terminals are: <MotorTerminals>. Please consult datatype for correct value.")

        if not isinstance(reverse, (MotorTerminals)):
            raise TypeError("Supported types for wheel terminals are: <MotorTerminals>. Please consult datatype for correct value.")

        self.forward = forward
        self.reverse = reverse

    def set_speed(self, pwm, duty) -> None:
        """Method for setting the speed and direction of a wheel."""

        if duty > 0:
            self.pwm.set_motor_pwm(self.reverse, 0)
            self.pwm.set_motor_pwm(self.forward, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(self.forward, 0)
            self.pwm.set_motor_pwm(self.reverse, abs(duty))

        else:
            self.pwm.set_motor_pwm(self.reverse, 0)
            self.pwm.set_motor_pwm(self.forward, 0)


class Motor:
    """
    This class contains methods for operating the motor of the vehicle (driving forward, backward
    and turning.). It contains the PCA9685 (PWM) for determining power to the DC motors through
    PWM duty cycles.

    The vehicle uses Differential Steering - a mechanism used to control the speed of independently
    powered wheels to turn the vehicle. During a turn, the wheels on the outer side of the turn must
    travel a longer distance than the inner wheels. The differential allows these wheels to rotate
    at different speeds, ensuring smooth and efficient turning without wheel slippage or strain on
    the drivetrain.

    Several methods are used to drive the vehicle. The set_motor_model() is the underlining method
    providing the most flexibility for defining speed and direction, and additional higher level
    methods (i.e. drive, reverse, drive_left, etc...) are available to simplify PWM duty logic.
    All of these methods require 12-bit (0-4095) signed integer values to power the motors for each
    wheel. For example:

    set_motor_model(2000, 2000, 2000, 2000) -> Forward
    set_motor_model(-2000, -2000, -2000, -2000) -> Backward
    set_motor_model(50, 50, 2000, 2000) -> Turn Left
    drive(2000) -> Forward (all 4 wheels)
    reverse_right(4095, 50) -> Reverse to the Right

    """

    __LEFT_UPPER_FORWARD = 0
    __LEFT_UPPER_REVERSE = 1
    __LEFT_LOWER_REVERSE = 2
    __LEFT_LOWER_FORWARD = 3
    __RIGHT_LOWER_FORWARD = 4
    __RIGHT_LOWER_REVERSE = 5
    __RIGHT_UPPER_FORWARD = 6
    __RIGHT_UPPER_REVERSE = 7


    def __init__(
        self,
        pwm_address: int = 0x40,
        pwm_frequency: int = 50
    ) -> None:

        if not isinstance(pwm_address, (int)):
            raise TypeError("Supported types for the PWM address are: <int>")

        if not isinstance(pwm_address, (int)):
            raise TypeError("Supported types for the PWM frequency are: <int>")

        self.pwm = PCA9685(pwm_address, pwm_frequency)
        self.wheel_left_lower = Wheel(forward=MotorTerminals.left_lower_forward, reverse=MotorTerminals.left_lower_reverse)


    def drive(self, pwm_duty: int) -> None:
        """
        Provide a positive PWM duty cycle to drive the vehicle forward
        by supplying consistent power to all 4 wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        pwm_duty = abs(pwm_duty)

        self.set_motor_model(pwm_duty, pwm_duty, pwm_duty, pwm_duty)


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

        self.set_motor_model(left_wheels_duty, left_wheels_duty, right_wheels_duty, right_wheels_duty)


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

        self.set_motor_model(left_wheels_duty, left_wheels_duty, right_wheels_duty, right_wheels_duty)


    def reverse(self, pwm_duty: int) -> None:
        """
        Provide a positive PWM duty cycle to reverse the vehicle
        by supplying consistent power to all 4 wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        pwm_duty = abs(pwm_duty)

        self.set_motor_model(-pwm_duty, -pwm_duty, -pwm_duty, -pwm_duty)


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

        self.set_motor_model(-left_wheels_duty, -left_wheels_duty, -right_wheels_duty, -right_wheels_duty)


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

        self.set_motor_model(-left_wheels_duty, -left_wheels_duty, -right_wheels_duty, -right_wheels_duty)


    def left_upper_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Upper Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(self.__LEFT_UPPER_REVERSE, 0)
            self.pwm.set_motor_pwm(self.__LEFT_UPPER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(self.__LEFT_UPPER_FORWARD, 0)
            self.pwm.set_motor_pwm(self.__LEFT_UPPER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(self.__LEFT_UPPER_FORWARD, 0)
            self.pwm.set_motor_pwm(self.__LEFT_UPPER_REVERSE, 0)


    def left_lower_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Lower Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(self.__LEFT_LOWER_REVERSE, 0)
            self.pwm.set_motor_pwm(self.__LEFT_LOWER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(self.__LEFT_LOWER_FORWARD, 0)
            self.pwm.set_motor_pwm(self.__LEFT_LOWER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(self.__LEFT_LOWER_FORWARD, 0)
            self.pwm.set_motor_pwm(self.__LEFT_LOWER_REVERSE, 0)


    def right_lower_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Right Lower Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(self.__RIGHT_LOWER_REVERSE, 0)
            self.pwm.set_motor_pwm(self.__RIGHT_LOWER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(self.__RIGHT_LOWER_FORWARD, 0)
            self.pwm.set_motor_pwm(self.__RIGHT_LOWER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(self.__RIGHT_LOWER_REVERSE, 0)
            self.pwm.set_motor_pwm(self.__RIGHT_LOWER_FORWARD, 0)


    def right_upper_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Right Upper Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(self.__RIGHT_UPPER_REVERSE, 0)
            self.pwm.set_motor_pwm(self.__RIGHT_UPPER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(self.__RIGHT_UPPER_FORWARD, 0)
            self.pwm.set_motor_pwm(self.__RIGHT_UPPER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(self.__RIGHT_UPPER_REVERSE, 0)
            self.pwm.set_motor_pwm(self.__RIGHT_UPPER_FORWARD, 0)


    def set_motor_model(
        self,
        left_upper_duty: int,
        left_lower_duty: int,
        right_upper_duty: int,
        right_lower_duty: int
    ) -> None:
        """
        Sets the speed and direction of the vehicle by providing PWM duty cycles
        for all of the wheels.
        """

        self.left_upper_wheel(left_upper_duty)
        self.left_lower_wheel(left_lower_duty)
        self.right_upper_wheel(right_upper_duty)
        self.right_lower_wheel(right_lower_duty)


    def stop(self) -> None:
        """
        Stops the vehicle by resetting the PWM duty cycle power supply.
        """
        self.set_motor_model(0, 0, 0, 0)
        self.pwm.close()


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
            time.sleep(0.5)

            # Pause
            self.set_motor_model(0, 0, 0, 0)
            time.sleep(0.5)

            self.reverse_right(3200, 300)
            time.sleep(0.5)

            # Pause
            self.set_motor_model(0, 0, 0, 0)
            time.sleep(0.5)


        self.set_motor_model(0, 0, 0, 0)



if __name__ == '__main__':

    motor = Motor()

    try:
        motor.set_motor_model(2000, 2000, 2000, 2000)  # Forward
        time.sleep(1)
        motor.set_motor_model(-2000, -2000, -2000, -2000)  # Back
        time.sleep(1)
        motor.set_motor_model(4000, 4000, 50, 50)  # Right
        time.sleep(0.5)
        motor.set_motor_model(50, 50, 4000, 4000)  # Left
        time.sleep(0.5)
        motor.turn_around(turn_cycles=3)    # Spin 180 degrees
        motor.stop()    # Stop

    except KeyboardInterrupt:
        motor.stop()