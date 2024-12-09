import math
import time
from PCA9685 import PCA9685
from ADC import Adc

class Motor:
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

    def left_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(0, 0)
            self.pwm.setMotorPwm(1, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(1, 0)
            self.pwm.setMotorPwm(0, abs(duty))
        else:
            self.pwm.setMotorPwm(0, 4095)
            self.pwm.setMotorPwm(1, 4095)

    def left_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(3, 0)
            self.pwm.setMotorPwm(2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(2, 0)
            self.pwm.setMotorPwm(3, abs(duty))
        else:
            self.pwm.setMotorPwm(2, 4095)
            self.pwm.setMotorPwm(3, 4095)

    def right_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(6, 0)
            self.pwm.setMotorPwm(7, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(7, 0)
            self.pwm.setMotorPwm(6, abs(duty))
        else:
            self.pwm.setMotorPwm(6, 4095)
            self.pwm.setMotorPwm(7, 4095)

    def right_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(4, 0)
            self.pwm.setMotorPwm(5, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(5, 0)
            self.pwm.setMotorPwm(4, abs(duty))
        else:
            self.pwm.setMotorPwm(4, 4095)
            self.pwm.setMotorPwm(5, 4095)

    def setMotorModel(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)

    def Rotate(self, n):
        angle = n
        bat_compensate = 7.5 / (self.adc.recvADC(2) * 3)
        while True:
            W = 2000

            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))

            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W

            PWM.setMotorModel(FL, BL, FR, BR)
            print("rotating")
            time.sleep(5 * self.rotation_delay = rotation_delay * bat_compensate / 1000)
            angle -= 5

    def test(self):
        self.setMotorModel(2000, 2000, 2000, 2000)  # Forward
        time.sleep(3)
        self.setMotorModel(-2000, -2000, -2000, -2000)  # Back
        time.sleep(3)
        self.setMotorModel(-500, -500, 2000, 2000)  # Left
        time.sleep(3)
        self.setMotorModel(2000, 2000, -500, -500)  # Right
        time.sleep(3)
        self.setMotorModel(0, 0, 0, 0)  # Stop

    def destroy(self):
        self.setMotorModel(0, 0, 0, 0)



# PWM = Motor()


# def loop():
#     PWM.setMotorModel(2000, 2000, 2000, 2000)  # Forward
#     time.sleep(3)
#     PWM.setMotorModel(-2000, -2000, -2000, -2000)  # Back
#     time.sleep(3)
#     PWM.setMotorModel(-500, -500, 2000, 2000)  # Left
#     time.sleep(3)
#     PWM.setMotorModel(2000, 2000, -500, -500)  # Right
#     time.sleep(3)
#     PWM.setMotorModel(0, 0, 0, 0)  # Stop


# def destroy():
#     PWM.setMotorModel(0, 0, 0, 0)


if __name__ == '__main__':
    motor = Motor()
    try:
        motor.test()
        # loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        # destroy()
        motor.destroy()