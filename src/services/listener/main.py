import paho.mqtt.client as mqtt
import os

# Set the MQTT broker and PORT
BROKER = "mqtt-broker"
PORT = 1883

# Set topics to subscribe to
TOPIC_SUBSCRIBE = [
    "device/infrared/left",
    "device/infrared/middle",
    "device/infrared/right"
]

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)


# Subscribes to topics in TOPIC_SUBSCRIBE
def on_connect(lClient, userdata, flags, return_code):
    if return_code == 0:
        for topic in TOPIC_SUBSCRIBE:
            client.subscribe(topic)
    else:
        print("could not connect, return code:", return_code)

# Edit logic here when receiving a message
def on_message(lClient, userdata, msg):
    message = msg.payload.decode()
    topic = msg.topic

    # Functionality can go here
    print(f"Received message: {message} on topic {topic}")

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