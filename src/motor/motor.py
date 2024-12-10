import math
import time
from enum import IntEnum
from PCA9685 import PCA9685
from ADC import Adc

class MotorChannels(IntEnum):
    """
    PWM channels for vehicle wheels used to determine direction of power supply.
    """
    LEFT_UPPER_FORWARD = 0
    LEFT_UPPER_REVERSE = 1
    LEFT_LOWER_REVERSE = 2
    LEFT_LOWER_FORWARD = 3
    RIGHT_LOWER_FORWARD = 4
    RIGHT_LOWER_REVERSE = 5
    RIGHT_UPPER_FORWARD = 6
    RIGHT_UPPER_REVERSE = 7


class Motor:
    """
    This class contains methods for operating the motor of the vehicle (driving forward, backward
    and turning.). It is comprised of the ADC and PCA9685 (PWM) for determining rotation angles and
    supplying power to the DC motors through PWM duty cycles.

    The vehicle uses Differential Steering - a mechanism used to control the speed of independently
    powered wheels to turn the vehicle. During a turn, the wheels on the outer side of the turn must
    travel a longer distance than the inner wheels. The differential allows these wheels to rotate
    at different speeds, ensuring smooth and efficient turning without wheel slippage or strain on
    the drivetrain.

    The vehicle is primarily driven through the set_motor_model which requires positive or negative
    integers representing the PWM duty cycle and direction (positive or negative values). For
    example:

    set_motor_model(2000, 2000, 2000, 2000) -> Forward
    set_motor_model(-2000, -2000, -2000, -2000) -> Backward
    set_motor_model(-500, -500, 2000, 2000) -> Turn Left

    """

    def __init__(
        self,
        rotation_delay: float = 2.5,
        pwm_address: int = 0x40,
        pwm_frequency: int = 50
    ) -> None:

        if not isinstance(rotation_delay, (float, int)):
            raise TypeError("Supported types for rotation delay are: <float>, <int>")

        if not isinstance(pwm_address, (int)):
            raise TypeError("Supported types for the PWM address are: <int>")

        if not isinstance(pwm_address, (int)):
            raise TypeError("Supported types for the PWM frequency are: <int>")


        self.rotation_delay = rotation_delay
        self.pwm = PCA9685(pwm_address, pwm_frequency)
        self.adc = Adc()


    def drive(self, pwm_duty: int) -> None:
        """
        Provide a positive PWM duty cycle to drive the vehicle forward
        by supplying consistent power to all 4 wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        pwm_duty = abs(pwm_duty)

        self.set_motor_model(pwm_duty, pwm_duty, pwm_duty, pwm_duty)


    def reverse(self, pwm_duty: int) -> None:
        """
        Provide a positive PWM duty cycle to reverse the vehicle
        by supplying consistent power to all 4 wheels.
        """

        # Ensure positive PWM duty cycle to handle direction
        pwm_duty = abs(pwm_duty)

        self.set_motor_model(-pwm_duty, -pwm_duty, -pwm_duty, -pwm_duty)


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


    @staticmethod
    def duty_range(duty1, duty2, duty3, duty4):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095

        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095

        if duty3 > 4095:
            duty3 = 4095
        elif duty3 < -4095:
            duty3 = -4095

        if duty4 > 4095:
            duty4 = 4095
        elif duty4 < -4095:
            duty4 = -4095
        return duty1, duty2, duty3, duty4


    def left_upper_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Upper Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(MotorChannels.LEFT_UPPER_REVERSE, 0)
            self.pwm.set_motor_pwm(MotorChannels.LEFT_UPPER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(MotorChannels.LEFT_UPPER_FORWARD, 0)
            self.pwm.set_motor_pwm(MotorChannels.LEFT_UPPER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(MotorChannels.LEFT_UPPER_FORWARD, 0)
            self.pwm.set_motor_pwm(MotorChannels.LEFT_UPPER_REVERSE, 0)


    def left_lower_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Lower Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(MotorChannels.LEFT_LOWER_REVERSE, 0)
            self.pwm.set_motor_pwm(MotorChannels.LEFT_LOWER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(MotorChannels.LEFT_LOWER_FORWARD, 0)
            self.pwm.set_motor_pwm(MotorChannels.LEFT_LOWER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(MotorChannels.LEFT_LOWER_FORWARD, 0)
            self.pwm.set_motor_pwm(MotorChannels.LEFT_LOWER_REVERSE, 0)


    def right_lower_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Right Lower Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_LOWER_REVERSE, 0)
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_LOWER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_LOWER_FORWARD, 0)
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_LOWER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_LOWER_REVERSE, 0)
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_LOWER_FORWARD, 0)


    def right_upper_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Right Upper Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_UPPER_REVERSE, 0)
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_UPPER_FORWARD, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_UPPER_FORWARD, 0)
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_UPPER_REVERSE, abs(duty))

        else:
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_UPPER_REVERSE, 0)
            self.pwm.set_motor_pwm(MotorChannels.RIGHT_UPPER_FORWARD, 0)


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

        left_u, left_l, right_u, right_l = self.duty_range(left_upper_duty, left_lower_duty, right_upper_duty, right_lower_duty)
        self.left_upper_wheel(left_u)
        self.left_lower_wheel(left_l)
        self.right_upper_wheel(right_u)
        self.right_lower_wheel(right_l)


    # def __rotate(self, n):
    #     """
    #     DO NOT USE YET: Method for turning the vehicle based on a given angle?
    #     """

    #     angle = n
    #     bat_compensate = 7.5 / (self.adc.recvADC(2) * 3)
    #     while True:
    #         W = 2000

    #         VY = int(2000 * math.cos(math.radians(angle)))
    #         VX = -int(2000 * math.sin(math.radians(angle)))

    #         FR = VY - VX + W
    #         FL = VY + VX - W
    #         BL = VY - VX - W
    #         BR = VY + VX + W

    #         PWM.setMotorModel(FL, BL, FR, BR)
    #         print("rotating")
    #         time.sleep(5 * self.rotation_delay = rotation_delay * bat_compensate / 1000)
    #         angle -= 5


    def stop(self) -> None:
        """
        Stops the vehicle by resetting the PWM duty cycle power supply.
        """
        self.set_motor_model(0, 0, 0, 0)
        self.pwm.close()



if __name__ == '__main__':
    motor = Motor()
    try:
        motor.set_motor_model(2000, 2000, 2000, 2000)  # Forward
        time.sleep(1)
        motor.set_motor_model(-2000, -2000, -2000, -2000)  # Back
        time.sleep(1)
        motor.set_motor_model(4000, 4000, 50, 50)  # Right
        time.sleep(2)
        motor.set_motor_model(50, 50, 4000, 4000)  # Left
        time.sleep(2)
        motor.stop()    # Stop

    except KeyboardInterrupt:
        motor.stop()