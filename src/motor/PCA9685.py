import time
import math
from smbus3 import SMBus


class PCA9685:
    """
    This class contains methods for the Raspi PCA9685 16-Channel, 12-bit, PWM Servo Driver
    and is primarily used for controlling the Motor, Servos, and LEDs. It communicates over I2C,
    supports PWM frequencies from 24 Hz to 1526 Hz, a clock input up to 50 MHz, and a 25 MHz
    internal oscillator.

    For typical use, devices will instantiate this class with options to specify a frequency,
    an I2C address, and the debug mode (future feature). By default, the frequency is set to
    50 Hz and the I2C address is 0x40 (`i2cdetect -y 1`).

    """

    # Registers/etc.
    __SUBADR1            = 0x02
    __SUBADR2            = 0x03
    __SUBADR3            = 0x04
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09
    __ALLLED_ON_L        = 0xFA
    __ALLLED_ON_H        = 0xFB
    __ALLLED_OFF_L       = 0xFC
    __ALLLED_OFF_H       = 0xFD

    def __init__(
          self,
          address: int = 0x40,
          frequency: int = 50,
          debug: bool = False
        ) -> None:

        self.bus = SMBus(1)
        self.address = address
        self.frequency = frequency
        self.debug = debug
        self.write(self.__MODE1, 0x00)
        self._set_pwm_freq(self.frequency)


    def read(self, register: int) -> int:
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, register)
        return result


    def _set_pwm_freq(self, freq: int) -> None:
        "Sets the PWM frequency. Default appears to be 50 Hz."

        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10        # sleep

        self.write(self.__MODE1, newmode)        # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)


    def set_pwm(self, channel: int, on: int, off: int) -> None:
        "Sets a single PWM channel"

        self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
        self.write(self.__LED0_ON_H+4*channel, on >> 8)
        self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
        self.write(self.__LED0_OFF_H+4*channel, off >> 8)


    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """Wrapper function for setting the motor PWM."""
        self.set_pwm(channel , 0, duty)


    def set_servo_pulse(self, channel: int, pulse: int) -> None:
        "Sets the Servo Pulse. The PWM frequency must be 50HZ."

        #PWM frequency is 50HZ,the period is 20000us
        pulse = pulse * 4096 / 20000
        self.set_pwm(channel, 0, int(pulse))


    def write(self, register: int, data: int) -> None:
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, register, data)
