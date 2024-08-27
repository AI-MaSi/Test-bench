import requests
import time
import random
import ADC_sensors
import PWM_controller
from PID import PID

SERVER_URL = 'http://192.168.0.131:8000'
LOOP_HZ = 20  # Control loop frequency in Hz


class SimpleEnv:
    def __init__(self):
        self.pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                                          inputs=2,
                                          simulation_mode=True,
                                          pump_variable=False,
                                          tracks_disabled=True,
                                          input_rate_threshold=0,
                                          deadzone=0)
        self.adc = ADC_sensors.ADC_hat(config_file='sensor_config.yaml',
                                       decimals=4,
                                       simulation_mode=True,
                                       min_sim_voltage=0.5,
                                       max_sim_voltage=4.5,
                                       frequency=0.1)

        self.current_angle = 0
        self.target_angle = 45
        self.button_state = 0
        self.min_angle = 0
        self.max_angle = 90

    def update_state(self):
        angles = self.adc.read_filtered(read="angle")
        self.current_angle = angles["LiftBoom angle"]
        return self.current_angle

    def read_pressures(self):
        pressures = self.adc.read_filtered(read="pressure")
        return pressures["Pump"], pressures["LiftBoom extension"], pressures["LiftBoom retraction"]

    def calibrate(self):
        print("Starting calibration...")
        self.adc.reset_angle_range()

        # Drive boom up
        self.pwm.update_values([0.070, 1])
        time.sleep(1)  # Wait for pressure to stabilize

        while True:
            current_angle, current_pressure = self.read_sensors()
            print(f"[UP] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
            if current_pressure >= 200:
                self.pwm.update_values([0.070, 0])  # Stop
                break
            time.sleep(0.1)

        # Drive boom down
        self.pwm.update_values([0.070, -1])
        time.sleep(1)  # Wait for pressure to stabilize

        while True:
            current_angle, current_pressure = self.read_sensors()
            print(f"[DOWN] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
            if current_pressure >= 200:
                self.pwm.update_values([0.070, 0])  # Stop
                break
            time.sleep(0.1)

        # Get calibrated range
        ranges = self.adc.get_angle_range()
        self.min_angle, self.max_angle = ranges["LiftBoom angle"]
        print(f"Calibrated angle range: {self.min_angle:.2f} to {self.max_angle:.2f}")

        # Move to midpoint
        mid_angle = (self.min_angle + self.max_angle) / 2
        print(f"Moving to mid point: {mid_angle:.2f}")

        while True:
            current_angle, _ = self.read_sensors()
            if abs(current_angle - mid_angle) < 1:  # 1 degree tolerance
                self.pwm.update_values([0.070, 0])  # Stop
                print(f"Arm positioned at middle angle: {current_angle:.1f}")
                break
            elif current_angle < mid_angle:
                self.pwm.update_values([0.070, -0.5])  # Move up slowly
            else:
                self.pwm.update_values([0.070, 0.5])  # Move down slowly
            time.sleep(0.1)

        print("Calibration complete!")

    def read_sensors(self):
        angles = self.adc.read_filtered(read="angle")
        pressures = self.adc.read_filtered(read="pressure")
        return angles["LiftBoom angle"], pressures["Pump"]


def send_values(data):
    try:
        response = requests.post(f'{SERVER_URL}/send_data/client2', json=data)
        if response.status_code == 200:
            return data
        else:
            print(f"Failed to send values. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")


def get_latest_values():
    try:
        response = requests.get(f'{SERVER_URL}/receive_data/client2')
        if response.status_code == 200:
            data = response.json()
            return data
        else:
            print(f"Failed to get values. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")


def generate_random_setpoint(min_angle, max_angle):
    return random.uniform(min_angle, max_angle)


if __name__ == "__main__":
    env = SimpleEnv()

    # Run calibration
    env.calibrate()

    pid_controller = PID(P=0.2, I=0.8, D=0.0)
    pid_controller.setSampleTime(1.0 / LOOP_HZ)
    pid_controller.setWindup(1.0)

    while True:
        # Get the latest values from the server
        received_data = get_latest_values()
        if received_data and 'values' in received_data and len(received_data['values']) > 7:
            env.target_angle = received_data['values'][7]  # The slider value (setpoint) is the 8th value
        else:
            # Generate a random setpoint if received data is None or invalid
            env.target_angle = generate_random_setpoint(env.min_angle, env.max_angle)
            print("Generated random setpoint:", env.target_angle)

        # Update the current state
        current_angle = env.update_state()

        # Calculate PID output
        pid_controller.SetPoint = env.target_angle
        pid_controller.update(current_angle)
        action = pid_controller.output

        # Clamp the output to [-1, 1]
        action = max(-1, min(1, action))

        # Flip output, as angle rises when servo goes down
        action = -action

        # Apply the action and get servo angles
        servo_angle = env.pwm.update_values([0.070, action], return_servo_angles=True)

        print(f"Servo angle: {servo_angle}")

        # Read pressures
        pump_pressure, boom_extraction_pressure, boom_retraction_pressure = env.read_pressures()

        # Toggle button state (for demonstration purposes)
        env.button_state = 1 - env.button_state

        # Prepare data to send back to the server
        send_data = {
            'values': [
                pump_pressure,
                boom_extraction_pressure,
                boom_retraction_pressure,
                action,  # pump control value
                servo_angle,
                current_angle,  # boom angle
                env.button_state,
                env.target_angle  # slider (setpoint)
            ]
        }

        # Send the data
        send_values(send_data)

        print(f"Target: {env.target_angle:.2f}, Current: {current_angle:.2f}, Action: {action:.2f}")

        # Wait for the next cycle
        time.sleep(1.0 / LOOP_HZ)
