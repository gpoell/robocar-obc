import time
from motor import Motor
from client.topics import MotorPublishers, MotorSubscribers, Topics
from client.mqtt_client import ClientConfig

def trajectory_planning(distance: int) -> int:

    if distance > 300: return 2000
    elif 200 < distance <= 300: return 1500
    elif 60 < distance <= 200: return 1000
    elif 20 < distance <= 60: return 600
    else: return 0


def main():
    """
    Future logic will operate motor based on State Machine states.

    Objective for Motor Behavior: Self Exploring
        1. While the motor is active, listen for distances published by
           the Ultrasonic sensor.
        2. Scale the PWM duty cycles for the wheels, slowing the vehicle
           as it approaches an obstacle or wall.
        3. Stop the vehicle for 3 seconds once it is within 30cm of the obstacle.
        4. Turn the vehicle ~180 degrees.

        Repeat steps 1-4 until vehicle reaches new obstacle
    """

    # Create Motor with topics and MQTT connection details
    publishers = MotorPublishers()
    subscribers = MotorSubscribers()
    topics = Topics(publishers, subscribers)
    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)
    motor = Motor(clientConfig)

    try:
        # Step 1: Collect distances
        while motor.state:
            time.sleep(0.10)    # Sample at 10 HZ
            distance = motor.target_distance

            # Step 2: Scale the PWM duty cycles for wheels
            wheels_power = trajectory_planning(distance)
            motor.drive(wheels_power)
            print(f"Distance to obstacle: {motor.target_distance} | PWM: {wheels_power}")

            # Step 3: Stop the vehicle if it reaches an obstacle
            if wheels_power == 0:
                # motor.stop()
                motor.drive(0)
                time.sleep(2)

                # Step 4: Turn the vehicle 180 degrees
                motor.turn_around(turn_cycles=3)
                time.sleep(1)

    except KeyboardInterrupt:
        print("Disconnecting motor...")
    except Exception as e:
        raise e("Error occured during Motor operation.")
    finally:
        motor.stop()
        motor._disconnect()
        print("Motor has stopped..")

main()