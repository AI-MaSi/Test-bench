import argparse
import requests
import time
import random
import ADC_sensors
import PWM_controller
from PID import PID
from model import Linear_QNet, QTrainer

import torch
from collections import deque
import numpy as np
import csv
import os

SERVER_URL = 'http://192.168.0.131:8000'
LOOP_HZ = 10  # Control loop frequency in Hz

# For agent
MAX_TIME_STEPS = 1000  # Maximum number of time steps per episode
MAX_MEMORY = 1_000_000
BATCH_SIZE = 1000
LR = 0.001

# New constants from pid_setter.py
TOLERANCE = 5  # +- degrees
TIME_GOAL = 2  # seconds needed to stay until new game
PUMP_IDLE = 0.060

class TestBenchEnv:
    def __init__(self, simulation):
        self.pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                                          inputs=2,
                                          simulation_mode=simulation,
                                          pump_variable=True,
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
        self.time_at_setpoint = 0

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
        self.min_angle, self.max_angle, min_raw, max_raw = ranges["LiftBoom angle"]
        print(f"Calibrated angle range: {self.min_angle:.2f} to {self.max_angle:.2f}")

        # Move to midpoint
        mid_angle = (self.min_angle + self.max_angle) / 2
        print(f"Moving to mid point: {mid_angle:.2f}")

        while True:
            current_angle, _ = self.read_sensors()
            if abs(current_angle - mid_angle) < TOLERANCE:  # 1 degree tolerance
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

    def step(self, action, pump_speed):
        # Apply the action
        self.pwm.update_values([pump_speed, action])

        # Update state
        new_angle = self.update_state()

        # Calculate reward
        reward = -abs(self.target_angle - new_angle) + 5  # +2 for possibility for pos reward :)

        # Check if done
        if abs(self.target_angle - new_angle) <= TOLERANCE:
            self.time_at_setpoint += 1 / LOOP_HZ
        else:
            # TODO: reset this in every new game!
            self.time_at_setpoint = 0

        done = float(self.time_at_setpoint >= TIME_GOAL)


        return new_angle, reward, done

class Agent:
    def __init__(self):
        self.n_games = 0
        self.epsilon = 0
        self.epsilon_min = 0.01     # test
        self.decay_rate = 0.01      # test
        self.gamma = 0.99
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet(3, 256, 2)  # Input: current_angle, target_angle, current_pressure; Output: action, pump_speed
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)

    def get_action(self, state):
        self.epsilon = max(0.1, 100 - self.n_games * 1.0)   # adjust 1.0
        #self.epsilon = max(0.1, self.epsilon_min + (1.0 - self.epsilon_min) * np.exp(-self.n_games / self.decay_rate))

        if random.random() < self.epsilon:
            return random.uniform(-1, 1), random.uniform(PUMP_IDLE, PUMP_IDLE + 0.05)
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model(state0)
            action = torch.tanh(prediction[0]).item()
            # sigmoid function converts the output to a value between 0 and 1
            pump_speed = PUMP_IDLE + torch.sigmoid(prediction[1]).item() * 0.05
            return action, pump_speed

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

    def pretrain(self, filename='pid_training_data.csv'):
        if not os.path.exists(filename):
            print(f"File '{filename}' not found. Skipping pretraining.")
            return  # Skip pretraining if the file doesn't exist

        with open(filename, 'r') as f:
            reader = csv.reader(f)
            header = next(reader, None)  # Read header
            data_exists = any(reader)  # Check if there's any data after the header

            if not data_exists:
                print(f"File '{filename}' only contains the header. Skipping pretraining.")
                return  # Skip pretraining if the file only contains the header

        # Reopen the file to actually read and process the data, as the first read exhausted the iterator
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header again
            for row in reader:
                # Skip the row if it is empty or has insufficient data
                if not row or len(row) < 7:
                    print(f"Skipping empty or incomplete row: {row}")
                    continue

                try:
                    current_angle, target_angle, action, pump_speed, reward, next_angle, done = map(float, row)
                    state = np.array([current_angle, target_angle, 0])  # Assume 0 pressure for pretraining
                    next_state = np.array([next_angle, target_angle, 0])
                    self.remember(state, (action, pump_speed), reward, next_state, done)
                except ValueError as e:
                    print(f"Error processing row: {row}")
                    print(f"Error message: {e}")
                    continue  # Skip this row and continue with the next
        self.train_long_memory()

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
    parser.add_argument('--mode', choices=['pid', 'rl_train', 'rl_run'], default='pid', help='Choose the mode to run')
    args = parser.parse_args()

    env = TestBenchEnv(simulation=args.simulation)
    agent = Agent()

    # Run calibration
    env.calibrate()

    pid_controller = PID(P=0.08, I=0.0, D=0.0)
    pid_controller.setSampleTime(1.0 / LOOP_HZ)
    pid_controller.setMaxOutput(1.0)
    pid_controller.setWindup(1.0)

    pump_pid_controller = PID(P=0.008, I=0.0, D=0.0)
    pump_pid_controller.setSampleTime(1.0 / LOOP_HZ)
    pump_pid_controller.setMaxOutput(0.05)
    pump_pid_controller.setWindup(1.0)


    # Ensure PID data file exists with header
    if not os.path.exists('pid_training_data.csv'):
        with open('pid_training_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['current_angle', 'target_angle', 'action', 'pump_speed', 'reward', 'next_angle', 'done'])

    # Pretrain RL with PID data if in RL mode
    if args.mode in ['rl_train', 'rl_run']:
        agent.pretrain()

    step_count = 0  # Initialize step counter
    setpoint_generated = False  # Flag to track if setpoint has been generated

    while True:
        if args.mode == 'rl_train':
            if not setpoint_generated:
                # Generate a random setpoint once per game
                env.target_angle = generate_random_setpoint(env.min_angle, env.max_angle)
                print("Generated random setpoint:", env.target_angle)
                setpoint_generated = True

            received_data = None
        else:
            received_data = None
            while received_data is None:
                received_data = get_latest_values()
                if received_data is None:
                    print("Waiting for valid data from server...")
                    time.sleep(1)

            if 'values' in received_data and len(received_data['values']) > 7:
                env.target_angle = received_data['values'][7]
                env.button_state = received_data['values'][6]
            else:
                env.target_angle = generate_random_setpoint(env.min_angle, env.max_angle)
                print("Generated random setpoint:", env.target_angle)

        # Initialize variables
        action = 0
        pump_speed = PUMP_IDLE
        reward = 0
        done = 0

        # Update the current state
        current_angle = env.update_state()
        _, current_pressure = env.read_sensors()

        if args.mode == 'pid' and env.button_state == 0:  # PID control when button is not pressed
            pid_controller.SetPoint = env.target_angle
            pid_controller.update(current_angle)
            action = max(-1, min(1, -pid_controller.output))

            pump_pid_controller.SetPoint = 0
            pump_pid_controller.update(abs(env.target_angle - current_angle))
            pump_speed = PUMP_IDLE + abs(pump_pid_controller.output)

            # Apply action and get next state
            next_angle, reward, done = env.step(action, pump_speed)

            # Save PID data for RL training
            save_pid_data([current_angle, env.target_angle, action, pump_speed, reward, next_angle, done])

        if args.mode == 'rl_train' or env.button_state == 1:
            state = np.array([current_angle, env.target_angle, current_pressure])
            action, pump_speed = agent.get_action(state)
            next_angle, reward, done = env.step(action, pump_speed)

            if args.mode == 'rl_train':  # Only train if in rl_train mode
                next_state = np.array([next_angle, env.target_angle, current_pressure])
                agent.remember(state, (action, pump_speed), reward, next_state, done)
                agent.train_short_memory(state, (action, pump_speed), reward, next_state, done)

                if done == 1.0:  # Check if the episode is complete
                    agent.n_games += 1  # Increment the game count only after an episode ends
                    agent.train_long_memory()  # Train long memory after each episode

                    if agent.n_games % 10 == 0:
                        agent.model.save()  # Save the model periodically

                    # Generate a new setpoint for the next episode
                    env.target_angle = generate_random_setpoint(env.min_angle, env.max_angle)
                    print("Generated new random setpoint after episode:", env.target_angle)

                    # Reset setpoint generation flag for the next game
                    setpoint_generated = True

            # Apply the action and get servo angles
            valve_angles = env.pwm.update_values([pump_speed, action], return_servo_angles=True)
            servo_angle = valve_angles['test_servo angle']
            # Read pressures
            pump_pressure, boom_extraction_pressure, boom_retraction_pressure = env.read_pressures()
            # Prepare data to send back to the server
            send_data = {
                'values': [
                    pump_pressure,
                    boom_extraction_pressure,
                    boom_retraction_pressure,
                    action,
                    servo_angle,
                    current_angle,
                    env.button_state,
                    env.target_angle
                ]
            }

            # dont send to server if training
            if args.mode != 'rl_train':
                # Send the data
                send_values(send_data)
        else:
            # For rl_run mode, we just need to apply the action and get next state
            valve_angles = env.pwm.update_values([pump_speed, action], return_servo_angles=True)
            servo_angle = valve_angles['test_servo angle']
            pump_pressure, boom_extraction_pressure, boom_retraction_pressure = env.read_pressures()
            send_data = {
                'values': [
                    pump_pressure,
                    boom_extraction_pressure,
                    boom_retraction_pressure,
                    action,
                    servo_angle,
                    current_angle,
                    env.button_state,
                    env.target_angle
                ]
            }
            send_values(send_data)

        # Show user the mode we are using
        selected_mode = "RL" if env.button_state else "PID"
        print(f"Mode: {selected_mode}, Game: {agent.n_games}, Epsilon: {agent.epsilon}, Target: {env.target_angle:.2f}, Current: {current_angle:.2f}, "
              f"Action: {action:.2f}, Pump Speed: {pump_speed:.3f}, Reward: {reward:.2f}")

        # Increment step count
        step_count += 1

        # Reset environment if step count exceeds MAX_TIME_STEPS
        if step_count > MAX_TIME_STEPS:
            print("Exceeded maximum time steps. Resetting environment.")
            step_count = 0
            agent.n_games += 1
            agent.time_at_setpoint = 0
            setpoint_generated = False  # Reset setpoint generation flag

        # Wait for the next cycle
        time.sleep(1.0 / LOOP_HZ)
