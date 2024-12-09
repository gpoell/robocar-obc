import math
import time
from PCA9685 import PCA9685
from ADC import Adc

class Motor:
    """
    This class contains methods for operating the motor of the vehicle (driving forward, backward
    and turning.). It is comprised of the ADC and PCA9685 (PWM) for determining rotation angles and
    supplying power to the DC motors through PWM duty cycles.

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
            self.pwm.set_motor_pwm(0, 0)
            self.pwm.set_motor_pwm(1, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(1, 0)
            self.pwm.set_motor_pwm(0, abs(duty))

        else:
            self.pwm.set_motor_pwm(0, 0)
            self.pwm.set_motor_pwm(1, 0)


    def left_lower_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Lower Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(3, 0)
            self.pwm.set_motor_pwm(2, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(2, 0)
            self.pwm.set_motor_pwm(3, abs(duty))

        else:
            self.pwm.set_motor_pwm(2, 0)
            self.pwm.set_motor_pwm(3, 0)


    def right_upper_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Upper Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(6, 0)
            self.pwm.set_motor_pwm(7, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(7, 0)
            self.pwm.set_motor_pwm(6, abs(duty))

        else:
            self.pwm.set_motor_pwm(6, 0)
            self.pwm.set_motor_pwm(7, 0)


    def right_lower_wheel(self, duty: int) -> None:
        """
        Determines the direction and sets the PWM duty cycle for the Left Upper Wheel
        """

        if duty > 0:
            self.pwm.set_motor_pwm(4, 0)
            self.pwm.set_motor_pwm(5, duty)

        elif duty < 0:
            self.pwm.set_motor_pwm(5, 0)
            self.pwm.set_motor_pwm(4, abs(duty))

        else:
            self.pwm.set_motor_pwm(4, 0)
            self.pwm.set_motor_pwm(5, 0)


    def set_motor_model(self, duty1: int, duty2: int, duty3: int, duty4: int) -> None:
        """
        Entry level method for setting the Motor Model, determining the speed and direction
        of the vehicle.
        """

        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)


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
        motor.set_motor_model(-2000, -2000, -2000, -2000)  # Forward
        time.sleep(1)
        motor.set_motor_model(2000, 2000, 2000, 2000)  # Back
        time.sleep(1)
        motor.set_motor_model(-4000, -4000, 2000, 2000)  # Right
        time.sleep(1)
        motor.set_motor_model(4000, 4000, -2000, -2000)  # Left
        time.sleep(2)
        motor.stop()    # Stop

    except KeyboardInterrupt:
        motor.stop()