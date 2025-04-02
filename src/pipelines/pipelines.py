class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value

        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        """Calculate PID output given the current system value and time step."""
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error

        return output


def obstacle_detection(ultra_sonice_distance: int) -> float:
    """
    Obstacle detection uses the distance recordings of the ultrasonic sensor.
    Returns a scaler based on the proximity to an object in front of the vehicle
    which is used for slowing down or speeding up.
    """
    if ultra_sonice_distance > 300: return 1.0
    elif 200 < ultra_sonice_distance <= 300: return 0.75
    elif 60 < ultra_sonice_distance <= 200: return 0.5
    elif 30 < ultra_sonice_distance <= 60: return 0.35
    else: return 0.0


def infra_lane_keep_pipeline(distance: int, lane_model: list, motor_model: list) -> tuple:

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


