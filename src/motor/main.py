import paho.mqtt.client as mqtt
from motor import Motor
from enum import Enum
from time import sleep

TOPICS = {
    "frontLeft": "device/motor/data/wheel/frontLeft",
    "frontRight": "device/motor/data/wheel/frontRight",
    "rearLeft": "device/motor/data/wheel/rearLeft",
    "rearRight": "device/motor/data/wheel/rearRight",
    "deviceStatus": "device/motor/status",
    "appStatus": "app/status",
    "sensorDistance": "device/ultrasonic/data"
}

TOPIC_SUBSCRIBE = "app/status"
DEVICE_STATUS = False

motor = Motor()

def trajectory_planning(distance: int) -> int:
    pwm = 0
    if distance > 300: pwm = 2000
    elif distance <= 300 and distance > 200: pwm = 1500
    elif distance <= 200 and distance > 100: pwm = 1000
    elif distance <= 100 and distance > 30: pwm = 600
    else: pwm = 0
    return pwm
    

# Define callbacks for MQTT
def on_connect(client, userdata, flags, return_code):
    if return_code == 0:
        print("Motor Connected.......")
        client.subscribe(TOPICS["appStatus"], qos=1)
        client.subscribe(TOPICS["sensorDistance"], qos=0)
    else:
        print("could not connect, return code:", return_code)

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    topic = msg.topic
    # print(f"Received message: {message} on topic {topic}")
    global DEVICE_STATUS
    if topic == TOPICS["appStatus"] and message != "active": DEVICE_STATUS = False
    if topic == TOPICS["sensorDistance"]:
        distance = int(float(message))
        motor.distance_to_target = distance
        # pwm = trajectory_planning(distance)
        # motor.setMotorModel(pwm, pwm, pwm, pwm)


if __name__ == "__main__":
    BROKER = "mqtt-broker"
    PORT = 1883


    try:
        client = mqtt.Client()

        client.on_connect = on_connect
        client.on_message = on_message

        client.connect(BROKER, PORT)
        client.loop_start()

        while True:
            sleep(0.5)
            pwm = trajectory_planning(motor.distance_to_target)
            print(f"Distance to Wall: {motor.distance_to_target} | PWM: {pwm}")
            motor.setMotorModel(-pwm, -pwm, -pwm, -pwm)

        # while True:
        #     data = sensor.get_distance()
        #     result = client.publish(TOPIC_PUBLISH_DATA, data)
        #     if result[0] != 0:
        #         print("Failed to send message to topic " + TOPIC_PUBLISH_DATA)
        #         if not client.is_connected():
        #             print("Client not connected, exiting...")
        #             break
    except KeyboardInterrupt:
        print("Disconnecting...")
    finally:
        # client.publish("device/ultrasonic/status", "off")
        client.loop_stop()
        client.disconnect()