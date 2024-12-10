import time
from enum import IntEnum, Enum
from statistics import median
from smbus3 import SMBus


class I2CAddress(IntEnum):
    CHANNEL = 1
    ADDRESS = 0X48
    REGISTER = 0xf4
    PCF8591_CMD = 0X40
    ADS7830_CMD = 0X84

class AdcChip(Enum):
    PCF8591 = "PCF8591"
    ADS7830 = "ADS7830"

class AdcChannel(IntEnum):
    LEFT_IDR = 0
    RIGHT_IDR = 1
    BATTERY = 2

class Adc:
    """
    The ADC module provides methods for reading/writing to the two ADC chips (PCF8591 and ADS7830)
    on the vehicle. Both chips are integrated along the same I2C bus and are used for returning
    the voltages of the left photoresistor, right photoresistor, and the battery. The I2C address
    can be obtained by running the `i2cdetect -y 1` command on the Raspberry Pi, and the ADC commands
    and registers are found within the respective datasheets.
    """

    def __init__(self) -> None:
        self.i2c_bus = SMBus(I2CAddress.CHANNEL)
        self.chip = AdcChip.ADS7830
        self.set_adc_chip()

    def set_adc_chip(self) -> None:
        """
        The ADS7830 chip is distinguished from the PCF8591 chip by reading the internal voltage: If it
        is lower than 150, it is the PCF8591 chip, otherwise it is the ADS7830 chip.
        """
        # Not clear why this is read multiple times. Appears to address a delay.
        for _ in range(5):
            data=self.i2c_bus.read_byte_data(I2CAddress.ADDRESS,I2CAddress.REGISTER)
            if data < 150:
                self.chip = AdcChip.PCF8591
            else:
                self.chip = AdcChip.ADS7830


    def read_pcf8591(self, channel: int, sample: int=9) -> int:
        """Return the median sample of bytes from the PCF8591 ADC channels: 0, 1, 2, 3"""
        value=[0]*sample
        for i in range(sample):
            value[i] = self.i2c_bus.read_byte_data(I2CAddress.ADDRESS, I2CAddress.PCF8591_CMD + channel)
        return median(value)


    def write_pcf8591(self, data: int) -> None:
        """Write DAC value to PCF8591 (there is a critical bug in original (open PR))"""
        self.i2c_bus.write_byte_data(I2CAddress.ADDRESS, 0, data)


    def recv_pcf8591(self, channel: int) -> int:
        """Reads the PCF8591 ADC value of channels: 0, 1, 2"""
        while True:
            value = self.read_pcf8591(channel)
            validate = self.read_pcf8591(channel)
            # Check for consistent ADC values
            if value == validate:
                break
        return value


    def recv_ads7830(self, channel: int) -> int:
        """Reads the ADS7830 ADC value of channels: 0, 1, 2"""

        # Set the Command based on the channel
        command = I2CAddress.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
        self.i2c_bus.write_byte(I2CAddress.ADDRESS, command)

        while True:
            value = self.i2c_bus.read_byte(I2CAddress.ADDRESS)
            validate = self.i2c_bus.read_byte(I2CAddress.ADDRESS)
            if value==validate:
                break
        return value


    def calculate_voltage(self, adc_val: int) -> float:
        """Calculates the voltage based on an ADC value from resistors"""
        result = adc_val / 255.0 * 3.3
        return round(result, 2)


    def voltage(self, channel: AdcChannel) -> float:
        """Returns the voltage from the ADC chip."""
        voltage = 0.0

        if self.chip == AdcChip.PCF8591:
            adc_data = self.read_pcf8591(channel.value)
            voltage = self.calculate_voltage(adc_data)

        elif self.chip == AdcChip.ADS7830:
            adc_data = self.recv_ads7830(channel.value)
            voltage = self.calculate_voltage(adc_data)

        return voltage


    def voltage_left_idr(self) -> float:
        """Returns the voltage for the LEFT photoresistor"""
        return self.voltage(AdcChannel.LEFT_IDR)


    def voltage_right_idr(self) -> float:
        """Returns the voltage for the RIGHT photoresistor"""
        return self.voltage(AdcChannel.RIGHT_IDR)


    def voltage_battery(self) -> float:
        """Returns the voltage for the battery"""
        # Scaling by 3 returns the overall battery voltage
        return self.voltage(AdcChannel.BATTERY) * 3


    def close(self) -> None:
        """Closes the I2C bus"""
        self.i2c_bus.close()


if __name__ == '__main__':
    import os
    if os.environ["ENVIRON"] == "PROD":
        adc = Adc()
        try:
            while True:
                left_idr = adc.voltage_left_idr()
                print ("The photoresistor voltage on the left is "+str(left_idr)+"V")
                right_idr = adc.voltage_right_idr()
                print ("The photoresistor voltage on the right is "+str(right_idr)+"V")
                battery = adc.voltage_battery()
                print ("The battery voltage is "+str(battery)+"V")
                time.sleep(1)
                print ('----')
        except Exception as e:
            print("There was an issue reading ADC values..")
            raise e
        finally:
            adc.close()
