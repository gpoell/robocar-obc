import importlib
# from gpiozero import LineSensor
from client.mqtt_client import MqttDevice, ClientConfig, Environment, State
from enum import IntEnum
from signal import pause



class IRPins(IntEnum):
    """
    Infrared Sensor Pins on vehicle.
    """
    IR01 = 14
    IR02 = 15
    IR03 = 23


class LineDetector(MqttDevice):
    """
    The LineDetector is positioned at the bottom of the vehicle and is used
    to detect when the vehicle crosses a line. It is composed of 3 infrared sensors
    that emit infrared light and detect the intensity of the light reflected back to
    the sensor, which is particularly useful for detecting lanes on contrasting surfaces
    to support autonomous lane navigation.

    The sensors functionality is encapsulated by the GIOZero LineSensor module and
    primarily report when the sensors detect a dark line through event based callbacks
    such as when_line() and when_no_line().

    PARAMETERS:

    :param samples:         The amount of samples each sensor collects. Default is 5.
    :param sample_rate:     The number of samples to read per second. Default is 100.
    :param threshold:       The threshold for determining an "active" state. Default is 0.5.
    """

    def __init__(
        self,
        client_config: ClientConfig,
        samples: int = 5,
        sample_rate: int = 100,
        threshold: float = 0.5
    ) -> None:

        if not isinstance(samples, int):
            raise TypeError("Supported types for samples are: <int>")
        if not isinstance(sample_rate, int):
            raise TypeError("Supported types for sample_rates are: <int>")
        if not isinstance(threshold, float):
            raise TypeError("Supported types for threshold are: <float>")
        if not isinstance(client_config, ClientConfig):
            raise TypeError("Supported types for client_config are: <ClientConfig>")
        if samples < 1 or sample_rate < 1:
            raise ValueError("Samples and sample rates must be positive values.")
        if 1 < threshold < 0:
            raise ValueError("Thresholds to determine active states must be between 0 and 1.")

        # Initialize MqttDevice
        super().__init__(client_config)

        # Import gpiozero if running on Raspberry Pi
        gpiozero = self.__import_gpiozero()


        # Infrared Sensors
        self.IR01 = gpiozero.LineSensor(pin=IRPins.IR01, queue_len=samples, sample_rate=sample_rate, threshold=threshold)
        self.IR02 = gpiozero.LineSensor(pin=IRPins.IR02, queue_len=samples, sample_rate=sample_rate, threshold=threshold)
        self.IR03 = gpiozero.LineSensor(pin=IRPins.IR03, queue_len=samples, sample_rate=sample_rate, threshold=threshold)

    def __import_gpiozero(self) -> None:
        """
        The gpiozero module must be imported on the Raspberry Pi which is determined by
        intentionally setting the system environment to PROD. This protects against
        running the containers on a separate device for testing purposes.
        """
        if self.env != Environment.PROD:
            raise ValueError("The LineDetector container is not running on a Raspberry Pi.")

        try:
            return importlib.import_module("gpiozero")
        except ImportError as e:
            print(f"Error importing library: {e}")
            raise e
        except Exception as e:
            raise e


    def line_detected(self) -> None:
        print(f"Line detected on {self.__class__.__name__}", self.value, sep=": ")

    def line_not_detected(self) -> None:
        print(f"Line not detected on {self.__class__.__name__}", self.value, sep=": ")


    def run(self) -> None:
        self.IR01.when_line = self.line_detected
        self.IR02.when_line = self.line_detected
        self.IR03.when_line = self.line_detected

        self.IR03.when_no_line = self.line_not_detected
        self.IR03.when_no_line = self.line_not_detected
        self.IR03.when_no_line = self.line_not_detected

        pause()

if __name__ == '__main__':

    sensor = LineDetector()

    try:
        sensor.run()
    except KeyboardInterrupt:
        print("Sensor is stopping...")

