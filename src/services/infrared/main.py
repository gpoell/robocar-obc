from infrared import IRLaneDetector
from client.topics import Topics, InfraredPublishers, InfraredSubscribers
from client.mqtt_client import ClientConfig

def main():

    publishers = InfraredPublishers()
    subscribers = InfraredSubscribers()
    topics = Topics(publishers, subscribers)
    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)

    ir_lane_detector = IRLaneDetector(client_config=clientConfig)

    try:
        ir_lane_detector.run(freq=0.05)
    except KeyboardInterrupt:
        print("Manual disconnect of Infrared Lane Detector...")
    except ValueError as e:
        print(e)
    finally:
        ir_lane_detector.stop()
        print("IR Lane Detector has stopped...")

main()