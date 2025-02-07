import time
from dataclasses import dataclass
from parts.PCA9685 import PCA9685


class MotorTerminal:
    """Wheel motors contain forward and reverse terminals."""

    def __init__(self, forward: int, reverse: int) -> None:
        self.forward = forward
        self.reverse = reverse


@dataclass(frozen=True)
class MotorTerminals:
    """Motor terminals for each wheel as specified by the datasheet."""

    left_upper: MotorTerminal = MotorTerminal(forward=0, reverse=1)
    left_lower: MotorTerminal = MotorTerminal(forward=3, reverse=2)
    right_lower: MotorTerminal = MotorTerminal(forward=4, reverse=5)
    right_upper: MotorTerminal = MotorTerminal(forward=6, reverse=7)


class Wheel:
    """
    Wheels are powered by DC motors that supply power to terminals
    to determine the speed and direction which they turn.

    @param  terminal:   the MotorTerminal containing forward and reverse terminals
    """

    def __init__(
        self,
        terminal: MotorTerminal,
    ) -> None:

        if not isinstance(terminal, (MotorTerminal)):
            raise TypeError("Supported types for wheel terminals are: <MotorTerminal>. Please consult datatype for correct value.")

        self.forward = terminal.forward
        self.reverse = terminal.reverse
        self.speed = 0

    def set_speed(self, pwm, duty) -> None:
        """Method for setting the speed and direction of a wheel."""

        if duty > 0:
            pwm.set_motor_pwm(self.reverse, 0)
            pwm.set_motor_pwm(self.forward, duty)

        elif duty < 0:
            pwm.set_motor_pwm(self.forward, 0)
            pwm.set_motor_pwm(self.reverse, abs(duty))

        else:
            pwm.set_motor_pwm(self.reverse, 0)
            pwm.set_motor_pwm(self.forward, 0)


class Motor:
    """
    The motor contains 4 wheels that are supplied power through the PCA9685 (PWM) to drive
    the vehicle by providing PWM duty cycles.

    The vehicle uses Differential Steering - a mechanism used to control the speed of independently
    powered wheels to turn the vehicle. During a turn, the wheels on the outer side of the turn must
    travel a longer distance than the inner wheels. The differential allows these wheels to rotate
    at different speeds, ensuring smooth and efficient turning without wheel slippage or strain on
    the drivetrain.

    The drive() method is primarily used to determine the speed and direction of the vehicle.
    It requires 4 float values between 0-1 to set the percentage of power supplied to each wheel.
    For example:

    drive(1, 1, 1, 1) -> Forward at full speed
    drive(-0.5, -0.5, -0.5, -0.5) -> Reverse at half speed
    drive(0.1, 0.1, 0.5, 0.5) -> Drive Left
    """

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
        self.wheel_left_lower = Wheel(terminal=MotorTerminals.left_lower)
        self.wheel_left_upper = Wheel(terminal=MotorTerminals.left_upper)
        self.wheel_right_lower = Wheel(terminal=MotorTerminals.right_lower)
        self.wheel_right_upper = Wheel(terminal=MotorTerminals.right_upper)


    def drive(self, l_u_duty: int, l_l_duty: int, r_u_duty: int, r_l_duty: int) -> None:
        """
        Sets the speed and direction of the vehicle by providing PWM duty cycles
        for all of the wheels.
        """

        self.wheel_left_upper.set_speed(pwm=self.pwm, duty=l_u_duty)
        self.wheel_left_lower.set_speed(pwm=self.pwm, duty=l_l_duty)
        self.wheel_right_upper.set_speed(pwm=self.pwm, duty=r_u_duty)
        self.wheel_right_lower.set_speed(pwm=self.pwm, duty=r_l_duty)


    def stop(self) -> None:
        """
        Stops the vehicle by resetting the PWM duty cycle power supply.
        """
        self.drive(0, 0, 0, 0)
        self.pwm.close()


if __name__ == '__main__':

    motor = Motor()

    try:
        motor.drive(2000, 2000, 2000, 2000)  # Forward
        time.sleep(1)
        motor.drive(-2000, -2000, -2000, -2000)  # Back
        time.sleep(1)
        motor.drive(4000, 4000, 50, 50)  # Right
        time.sleep(0.5)
        motor.drive(50, 50, 4000, 4000)  # Left
        time.sleep(0.5)
        motor.stop()    # Stop

    except KeyboardInterrupt:
        motor.stop()