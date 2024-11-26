import paho.mqtt.client as mqtt
import os

BROKER = "mqtt-broker"
PORT = 1883
TOPIC_PUBLISH = "app/status"
TOPIC_SUBSCRIBE = "device/ultrasonic/distance"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)


# Define callbacks
def on_connect(lClient, userdata, flags, return_code):
    if return_code == 0:
        print("State Machine connected.....")
        client.subscribe(TOPIC_SUBSCRIBE)
        client.subscribe("device/ultrasonic/status")
        client.subscribe("device/ultrasonic/threads")
    else:
        print("could not connect, return code:", return_code)

def on_message(lClient, userdata, msg):
    message = msg.payload.decode()
    topic = msg.topic
    print(f"Received message: {message} on topic {topic}")
    if topic == "device/ultrasonic/status" and message == "off":
        print("Ultrasonic sensor has disconnected...")
    
    # client.publish(TOPIC_PUBLISH, "active")

client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT)
client.loop_start()

try:
    client.publish("device/ultrasonic/status", "on")
    if os.environ["ENVIRON"] == "PROD":
        print("prod environment is active..")
    print(os.environ["ENVIRON"])
    while True:
        pass
except KeyboardInterrupt:
    print("Disconnecting...")
finally:
    client.publish("device/ultrasonic/status", "off")
    client.loop_stop()
    client.disconnect()