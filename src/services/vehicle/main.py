import time
from vehicle import Vehicle
from client.topics import VehiclePublishers, VehicleSubscribers, Topics
from client.mqtt_client import ClientConfig
from pipelines.pipelines import PID, obstacle_detection

def trajectory_planning(distance: int) -> int:

    if distance > 300: return 2000
    elif 200 < distance <= 300: return 1500
    elif 60 < distance <= 200: return 1000
    elif 20 < distance <= 60: return 600
    else: return 0


def obstacle_detection(distance: int) -> bool:
    """
    Simple obstacle detection.
    """
    return True if distance < 30 else False


infra_gap_count = 0


def lane_keep_pipeline(distance: int, lane_model: list, motor_model: list) -> tuple:

    new_motor = motor_model.copy()
    global infra_gap_count

    # Stop if there is an obstacle
    if obstacle_detection(distance): return [0, 0, 0, 0]

    # Turn hard right
    if lane_model[1] or lane_model[2]:
        new_motor[0] -= 10
        new_motor[1] -= 10
        new_motor[2] -= 20
        new_motor[3] -= 20
        infra_gap_count = 0
        return new_motor


    # Turn slow right
    if lane_model[0]:
        new_motor[0] += 20
        new_motor[1] += 20
        new_motor[2] -= 10
        new_motor[3] -= 10
        infra_gap_count = 0
        return new_motor

    # Drive straight if no line
    if 1 not in lane_model: return [500, 500, 500, 500]

    return motor_model


def main():
    """
    Objective for Vehicle Behavior: Lane Navigation
        1. Collect ultrasonic and IR lane detector data
        2. Return motor model based on lane keeping pipeline
        3. Set the vehicle speed and direction
        Repeat steps until vehicle is stopped.
    """

    # Create Motor with topics and MQTT connection details
    publishers = VehiclePublishers()
    subscribers = VehicleSubscribers()
    topics = Topics(publishers, subscribers)
    clientConfig = ClientConfig(topics, host="mqtt-broker", port=1883)
    vehicle = Vehicle(clientConfig)

    # Temporary motor model for driving vehicle
    motor_model = [0, 0, 0, 0]

    try:
        while vehicle.state:
            time.sleep(0.005)    # Sample at 50 HZ
            # Step 1: Collect sensor data
            distance = vehicle.target_distance
            lane_model = vehicle.lane_model

            # Step 2: Scale the PWM duty cycles for wheels
            mm = lane_keep_pipeline(distance, lane_model, motor_model)
            motor_model = mm

            # Step 3: Set motor speed
            vehicle.drive(motor_model[0], motor_model[1], motor_model[2], motor_model[3])
            print(f"Obstacle Distance: {vehicle.target_distance} | Lane Model: {lane_model} | PWM: {motor_model}")

            # # Stop Motor if there is an obstacle or if it falls off track
            # if sum(motor_model) == 0:
            #     break

    except KeyboardInterrupt:
        print("Vehicle was manually stopped..")
    except Exception as e:
        raise e("Error occured during Vehicle operation.")
    finally:
        vehicle.stop()
        print("Vehicle has stopped..")

main()