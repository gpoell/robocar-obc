from dataclasses import dataclass
from typing import Union

class Topic:
    """
    This is the base data type for MQTT Topics, ensuring correct properties are
    defined for publishing and subscribing data. All topics should contain
    the following:

    1. topic: a string representation of a topic in the format /my/topic/data
    2. qos: quality of service level between 0 and 3.
    """

    def __init__(
        self,
        topic: str,
        qos: int
    ) -> None:


        if not isinstance(topic, str):
            raise TypeError("Supported types for topic are: <str>")
        if not isinstance(qos, int):
            raise TypeError("Supported types for qos are: <int>")
        if qos > 3 or qos < 0:
            raise ValueError("MQTT clients only support QOS levels 0-3")

        self.topic = topic
        self.qos = qos


    def get_values(self) -> tuple:
        """Returns topic values."""
        vals = [val for attr, val in self.__dict__.items()]
        return tuple(vals)


@dataclass(frozen=True)
class Publishers:
    """Abstract class for Publisher data types."""


@dataclass(frozen=True)
class Subscribers:
    """Abstract class for Subscriber data types."""


class Topics:
    """
    A collection of topics intended for publishing and subscribing to the MQTT broker.

    :param publishers: defines <Publishing> topics for the respective device.
    :param subscribers: defines <Subscriber> topics for the respective device.
    """
    def __init__(
        self,
        publishers: Union[Publishers, None],
        subscribers: Union[Subscribers, None]
    ) -> None:

        assert isinstance(publishers, Publishers) or publishers is None,"Supported types of publishers are: Publishers, None"
        assert isinstance(subscribers, Subscribers) or subscribers is None, "Supported types of subscribers are: Subscribers, None"

        self.publishers = publishers
        self.subscribers = subscribers


@dataclass(frozen=True)
class UltrasonicPublishers(Publishers):
    """
    Ultrasonic topics for publishing data. Use this data type for
    creating the Topics object required by the MqttDevice class.

    - distance: publishes ultrasonic distance
    - status:   publishes the status of the ultrasonic sensor
    - threads:  publishes the current thread count
    """

    distance: Topic = Topic(topic="device/ultrasonic/distance", qos=0)
    status: Topic = Topic(topic="device/ultrasonic/status", qos=0)
    threads: Topic = Topic(topic="device/ultrasonic/threads", qos=0)


@dataclass(frozen=True)
class UltrasonicSubscribers(Subscribers):
    """
    Ultrasonic topics for subscribing to data. Use this data type for
    creating the Topics object required by the MqttDevice class.

    - appStatus: subscribes to the overall app status
    """

    appStatus: Topic = Topic(topic="app/status", qos=1)


@dataclass(frozen=True)
class MotorPublishers(Publishers):
    """
    Motor topics for publishing data. Use this data type for
    creating the Topics object required by the MqttDevice class.

    - luPWM:    publishes the PWM duty cycle for the Left Upper Wheel
    - llPWM:    publishes the PWM duty cycle for the Left Lower Wheel
    - ruPWM:    publishes the PWM duty cycle for the Right Upper Wheel
    - rlPWM:    publishes the PWM duty cycle for the Right Lower Wheel
    - status:   publishes the status of the ultrasonic sensor
    - threads:  publishes the current thread count
    """

    luPWM: Topic = Topic(topic="device/motor/wheel/left/upper/pwm", qos=0)
    llPWM: Topic = Topic(topic="device/motor/wheel/left/lower/pwm", qos=0)
    ruPWM: Topic = Topic(topic="device/motor/wheel/right/upper/pwm", qos=0)
    rlPWM: Topic = Topic(topic="device/motor/wheel/right/lower/pwm", qos=0)
    status: Topic = Topic(topic="device/motor/status", qos=0)
    threads: Topic = Topic(topic="device/motor/threads", qos=0)


@dataclass(frozen=True)
class MotorSubscribers(Subscribers):
    """
    Motor topics for subscribing to data. Use this data type for
    creating the Topics object required by the MqttDevice class.

    - appStatus: subscribes to the overall app status
    """

    appStatus: Topic = Topic(topic="app/status", qos=1)
    ultrasonicDistance: Topic = Topic(topic=UltrasonicPublishers.distance, qos=1)


def get_topics(topics: Union[Publishers, Subscribers]) -> list[tuple]:
    """
    Returns a list of topic values for Publishers or Subscribers.
    Useful for subscribing/publishing multiple topics to MQTT broker.
    """

    if not isinstance(topics, (Publishers, Subscribers)):
        raise TypeError("Supported types for get_topics are: <Publishers>, <Subscribers>")

    topics = [val for _, val in topics.__dict__.items()]
    return [topic.get_values() for topic in topics]
