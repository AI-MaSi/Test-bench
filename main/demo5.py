import argparse
import requests
import time
import random
import ADC_sensors
import PWM_controller
from PID import PID
from model import Linear_QNet, QTrainer

import torch
# import torch.nn.functional as F
from collections import deque
import numpy as np
import csv
import os

SERVER_URL = 'http://192.168.0.131:8000'
LOOP_HZ = 10  # Control loop frequency in Hz

# For agent
TIME_STEPS = 200  # Maximum number of time steps per episode
MAX_MEMORY = 1_000_000
BATCH_SIZE = 1000
LR = 0.001


class TestBenchEnv:
    def __init__(self,simulation):
        self.pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                                          inputs=2,
                                          simulation_mode=simulation,
                                          pump_variable=False,
                                          tracks_disabled=True,
                                          input_rate_threshold=0,
                                          deadzone=0)
        self.adc = ADC_sensors.ADC_hat(config_file='sensor_config.yaml',
                                       decimals=4,
                                       simulation_mode=simulation,
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

    def step(self, action):
        # flip the action so it matches PID
        action = -action

        # Apply the action
        self.pwm.update_values([0.070, action])

        # Update state
        new_angle = self.update_state()

        # Calculate reward
        reward = -abs(self.target_angle - new_angle)

        # Check if done
        done = abs(self.target_angle - new_angle) < 2  # Within 1 degree

        return new_angle, reward, done


class Agent:
    def __init__(self):
        self.n_games = 0
        self.epsilon = 0
        self.gamma = 0.99
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet(3, 256, 1)  # Input: current_angle, target_angle, current_pressure; Output: action
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)

    def get_action(self, state):
        self.epsilon = max(0.1, 100 - self.n_games * 0.1)  # Decays more slowly and never goes below 0.1
        if random.random() < self.epsilon:
            return random.uniform(-1, 1)
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model(state0)
            return torch.tanh(prediction).item()  # Use tanh to bound the output between -1 and 1

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train_short_memory(self, state, action, reward, next_state, done):
        self.trainer.train_step(state, action, reward, next_state, done)

    def train_long_memory(self):
        if len(self.memory) > BATCH_SIZE:
            mini_sample = random.sample(self.memory, BATCH_SIZE)
        else:
            mini_sample = self.memory

        states, actions, rewards, next_states, dones = zip(*mini_sample)
        self.trainer.train_step(states, actions, rewards, next_states, dones)


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


def save_pid_data(data, filename='pid_training_data.csv'):
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Run agent in simulation mode')
    parser.add_argument('--simulation', action='store_true', help='Run the script in simulation mode')
    args = parser.parse_args()

    # Pass the simulation argument to TestBenchEnv
    env = TestBenchEnv(simulation=args.simulation)  # Pass the argument to TestBenchEnv
    agent = Agent()

    # Run calibration
    env.calibrate()

    pid_controller = PID(P=0.05, I=0.2, D=0.0)
    pid_controller.setSampleTime(1.0 / LOOP_HZ)

    # set PID to output between -1..1
    pid_controller.setMaxOutput(1.0)

    pid_controller.setWindup(1.0)

    # Ensure PID data file exists with header
    if not os.path.exists('pid_training_data.csv'):
        with open('pid_training_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['current_angle', 'target_angle', 'action', 'reward', 'next_angle', 'done'])

    while True:
        # Get the latest values from the server
        received_data = None
        while received_data is None:
            received_data = get_latest_values()
            if received_data is None:
                print("Waiting for valid data from server...")
                time.sleep(1)  # Wait for a second before trying again

        if 'values' in received_data and len(received_data['values']) > 7:
            env.target_angle = received_data['values'][7]  # The slider value (setpoint) is the 8th value
            env.button_state = received_data['values'][6]  # The button state is the 7th value
        else:
            # Generate a random setpoint if received data is invalid
            env.target_angle = generate_random_setpoint(env.min_angle, env.max_angle)
            print("Generated random setpoint:", env.target_angle)


        # Update the current state
        current_angle = env.update_state()
        _, current_pressure = env.read_sensors()

        # Decide between PID and RL based on button state
        if env.button_state == 0:  # PID control
            pid_controller.SetPoint = env.target_angle
            pid_controller.update(current_angle)
            action = pid_controller.output
            action = max(-1, min(1, -action))  # Clamp and flip output

            # Apply action and get next state
            next_angle, reward, done = env.step(action)

            # Save PID data for RL training
            save_pid_data([current_angle, env.target_angle, action, reward, next_angle, done])

        else:  # RL control
            state = np.array([current_angle, env.target_angle, current_pressure])
            action = agent.get_action(state)
            next_angle, reward, done = env.step(action)

            next_state = np.array([next_angle, env.target_angle, current_pressure])
            agent.remember(state, action, reward, next_state, done)
            agent.train_short_memory(state, action, reward, next_state, done)

        # Apply the action and get servo angles
        valve_angles = env.pwm.update_values([0.070, action], return_servo_angles=True)

        servo_angle = valve_angles['test_servo angle']

        #print(f"Servo angle: {servo_angle}")

        # Read pressures
        pump_pressure, boom_extraction_pressure, boom_retraction_pressure = env.read_pressures()

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

        print(
            f"Mode: {'RL' if env.button_state else 'PID'}, Target: {env.target_angle:.2f}, Current: {current_angle:.2f}, Action: {action:.2f}, Reward: {reward:.2f}")

        # Train long memory and save model periodically
        if env.button_state == 1:  # Only for RL mode
            agent.n_games += 1
            if agent.n_games % 100 == 0:
                agent.train_long_memory()
                agent.model.save()

        # Wait for the next cycle
        time.sleep(1.0 / LOOP_HZ)