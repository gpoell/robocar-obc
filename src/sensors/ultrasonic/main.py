from ultrasonic import Ultrasonic
from devices import Topic, Publishers, Subscribers, Topics, ClientConfig

def main():
    
    # Define Topics to publish and subscribe
    publishers = Publishers()
    publishers.distance = Topic(topic="device/ultrasonic/distance", qos=0)
    publishers.status = Topic(topic="device/ultrasonic/status", qos=0)
    publishers.threads = Topic(topic="device/ultrasonic/threads", qos=0)
    
    subscribers = Subscribers()
    subscribers.appStatus = Topic(topic="app/status", qos=1)

    topics = Topics(publishers, subscribers)

    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)

    print('Ultrasonic Sensor is initializing ... ')
    ultrasonic = Ultrasonic(clientConfig, trig=27, echo=22, maxDistance=300)

    try:
        ultrasonic.run(freq=0.05)
    except KeyboardInterrupt:
        print("Disconnecting...")
    except ValueError:
        raise ValueError
    finally:
        ultrasonic.stop()

main()