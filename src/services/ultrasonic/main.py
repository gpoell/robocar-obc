from parts.ultrasonic import Ultrasonic
from client.topics import Topics, UltrasonicPublishers, UltrasonicSubscribers
from client.mqtt_client import ClientConfig

def main():

    publishers = UltrasonicPublishers()
    subscribers = UltrasonicSubscribers()
    topics = Topics(publishers, subscribers)
    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)
    ultrasonic = Ultrasonic(clientConfig, trig=27, echo=22)

    try:
        ultrasonic.run(freq=0.05)
    except KeyboardInterrupt:
        print("Disconnecting...")
    except ValueError as e:
        raise ValueError from e
    finally:
        ultrasonic.stop()
        print("Ultrasonic sensor stopped..")

main()