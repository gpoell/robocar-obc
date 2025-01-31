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
        client.subscribe("device/infrared/left")
        client.subscribe("device/infrared/middle")
        client.subscribe("device/infrared/right")
        client.subscribe("device/infrared/status")
        client.subscribe("device/infrared/threads")
    else:
        print("could not connect, return code:", return_code)

def on_message(lClient, userdata, msg):
    message = msg.payload.decode()
    topic = msg.topic
    print(f"Received message: {message} on topic {topic}")
    if topic == "device/infrared/status" and message == "off":
        print("Infrared sensor has disconnected...")

    # client.publish(TOPIC_PUBLISH, "active")

client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT)
client.loop_start()

try:
    # client.publish("device/ultrasonic/status", "on")
    print("Environment", os.environ["ENVIRON"], sep=": ")
    print("Listening for topics...")
    while True:
        pass
except KeyboardInterrupt:
    print("Disconnecting...")
finally:
    # client.publish("device/ultrasonic/status", "off")
    client.loop_stop()
    client.disconnect()