"""
For training: python3 agent2.py --mode train
For RL without training: python3 agent2.py --mode rl
For PID control: python3 agent2.py --mode pid
For interactive mode: python3 agent2.py --mode interactive
"""

import requests
#import json
import threading
import torch
import torch.nn.functional as F
import random
import numpy as np
import time
import argparse
import os
import csv
import sys
import select
from collections import deque

from model import Linear_QNet, QTrainer
import matplotlib.pyplot as plt

from ADC_sensors import ADC_hat
from PWM_controller import PWM_hat
from PID import PID

# TODO: adjust pump value during training to get the movement be smoother
# TODO: multiple plots for comparing PID/RL
# TODO: stuff
# ----------------------------------------

LOOP_HZ = 20  # Control loop frequency in Hz
TIME_STEPS = 200  # Maximum number of time steps per episode

MAX_MEMORY = 1_000_000  # Increased from 100,000
BATCH_SIZE = 1000
LR = 0.001

# not used yet
FULLY_TRAINED = 10000    # points to stop training early


class XRCommunicator:
    def __init__(self, env, server_url='http://localhost:8000'):
        self.env = env
        self.server_url = server_url
        self.latest_data = {
            'pump_pressure': 0,
            'boom_extraction_pressure': 0,
            'boom_retraction_pressure': 0,
            'pump_control_value': 0,
            'servo_angle': 0,
            'boom_angle': 0,
            'control_mode': '0',  # 'PID 0' or 'RL 1'
            'target_angle': 45  # Default to midpoint
        }
        self.lock = threading.Lock()
        self.running = True

    def start(self):
        self.send_thread = threading.Thread(target=self.send_data_loop)
        self.receive_thread = threading.Thread(target=self.receive_data_loop)
        self.send_thread.start()
        self.receive_thread.start()

    def stop(self):
        self.running = False
        self.send_thread.join()
        self.receive_thread.join()

    def send_data_loop(self):
        while self.running:
            try:
                self.update_local_data()
                data = {'values': list(self.latest_data.values())}
                response = requests.post(f'{self.server_url}/send_data/client2', json=data, timeout=1)
                if response.status_code != 200:
                    print(f"Failed to send data. Status code: {response.status_code}")
            except requests.exceptions.RequestException as e:
                print(f"Send request failed: {e}")
            time.sleep(0.05)  # 50ms delay

    def receive_data_loop(self):
        while self.running:
            try:
                response = requests.get(f'{self.server_url}/receive_data/client2', timeout=1)
                if response.status_code == 200:
                    received_data = response.json()
                    self.update_received_data(received_data)
                else:
                    print(f"Failed to receive data. Status code: {response.status_code}")
            except requests.exceptions.RequestException as e:
                print(f"Receive request failed: {e}")
            time.sleep(0.05)  # 50ms delay

    def update_local_data(self):
        with self.lock:
            self.latest_data['pump_pressure'] = self.env.adc.read_pressure()[0]
            self.latest_data['boom_extraction_pressure'] = self.env.adc.read_pressure()[1]
            self.latest_data['boom_retraction_pressure'] = self.env.adc.read_pressure()[2]
            self.latest_data['pump_control_value'] = self.env.pwm.values[0]
            _, servo_angle = self.env.pwm.update_values([0.070, self.env.pwm.values[1]], return_servo_angles=True)
            self.latest_data['servo_angle'] = servo_angle
            self.latest_data['boom_angle'] = self.env.current_angle

    def update_received_data(self, received_data):
        with self.lock:
            if 'control_mode' in received_data:
                self.latest_data['control_mode'] = received_data['control_mode']
            if 'target_angle' in received_data:
                self.latest_data['target_angle'] = received_data['target_angle']

    def get_latest_data(self):
        with self.lock:
            return self.latest_data.copy()


def run_xr_mode(env, agent, pid_controller):
    communicator = XRCommunicator(env)
    communicator.start()

    try:
        while True:
            data = communicator.get_latest_data()

            # Update target angle based on slider value
            env.target_angle = data['slider']

            # Choose between RL and PID based on button state
            if data['button'] == 1:
                # Use RL
                state = env.update_state()
                action = agent.get_action(state)
            else:
                # Use PID
                current_angle = env.current_angle
                pid_controller.SetPoint = env.target_angle
                pid_controller.update(current_angle)
                action = pid_controller.output
                action = max(-1, min(1, action))  # Clamp to [-1, 1]
                action = -action  # Flip output

            # Apply the action
            next_state, reward, done = env.step(action)

            # Update the pump control value in the data
            data['pump_control_value'] = env.pwm.values[0]

            time.sleep(1 / LOOP_HZ)  # Maintain the loop frequency

    except KeyboardInterrupt:
        print("XR mode stopped.")
    finally:
        communicator.stop()


class TestBenchEnv:
    def __init__(self, sensor, pwm_driver, max_angle, min_angle):
        if sensor is None or pwm_driver is None:
            raise ValueError("Sensor and PWM driver must be initialized before creating the environment!")

        # boltzmann exploration does not work with single value output
        self.exploration_type = "epsilon-greedy"  # or "boltzmann"


        self.reward_zone = 10.0  # Reward zone +- in degrees

        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle_range = max_angle - min_angle

        self.pwm = pwm_driver
        self.adc = sensor

        self.current_angle = 0  # 0.1

        self.max_pressure = 300  # Maximum pressure value

        self.time_at_target = 0
        self.max_time_steps = TIME_STEPS
        self.current_time_step = 0
        self.dt = 1 / LOOP_HZ  # Time step in seconds
        self.last_update_time = time.time()
        self.debug_mode = False

        self.target_angle = self.generate_random_angle()

    def generate_random_angle(self):
        if self.min_angle is None or self.max_angle is None:
            raise ValueError("Min and max angles must be set before generating a random angle")
        return random.uniform((self.min_angle+15), (self.max_angle-15))

    def get_values_from_server(self):
        # TODO: use XRCommunicator to get values
        pass

    def send_values_to_server(self):
        # TODO: use XRCommunicator to send values
        pass

    def reset(self):
        self.target_angle = self.generate_random_angle()
        self.time_at_target = 0
        self.current_time_step = 0
        return self.update_state()

    def step(self, action):
        reward = 0

        # Ensure the loop runs at the specified frequency
        current_time = time.time()
        time_to_sleep = self.dt - (current_time - self.last_update_time)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)
        self.last_update_time = time.time()

        # Update PWM with the action
        self.pwm.update_values([0.070, action])

        # Update time step
        self.current_time_step += 1

        # Update state
        self.state = self.update_state()

        # Extract current angle from state
        self.current_angle = self.state[0] * self.max_angle

        # Calculate the current distance to the target angle (signed distance)
        current_distance = self.current_angle - self.target_angle


        # distance penalty
        reward -= abs(current_distance) / (10 * TIME_STEPS)

        # Reward for maintaining target angle within the reward zone
        done = False
        if abs(current_distance) < self.reward_zone:
            self.time_at_target += 1
            reward += 0.1  # Additional reward for maintaining target angle

            if self.time_at_target >= 20:
                reward += 20  # Big reward
                done = True
                self.reset()
        else:
            self.time_at_target = 0

        # Terminate if the maximum time steps are exceeded
        if self.current_time_step >= self.max_time_steps:
            done = True

        # Debugging output
        if self.debug_mode:
            self.debug_print(reward, action)


        # Update the state with the current pressure
        self.state = np.array([
            self.current_angle / self.max_angle,
            current_distance / self.max_angle,
            self.current_pressure / self.max_pressure
        ])

        return self.state, reward, done

    def update_state(self):
        # Read the LiftBoom angle
        angles = self.adc.read_filtered(read="angle")
        self.current_angle = angles["LiftBoom angle"]

        # Read the current pump pressure
        pressures = self.adc.read_filtered(read="pressure")
        self.current_pressure = pressures["Pump"]

        # Calculate the current distance to the target angle
        current_distance = self.current_angle - self.target_angle

        # Construct the state with normalized values
        """
        self.state = np.array([
            self.current_angle / self.max_angle,  # Normalized current angle
            current_distance / self.max_angle,  # Normalized distance to target
            self.time_at_target / self.max_time_steps,  # Normalized time spent in target zone
            self.current_pressure / self.max_pressure  # Normalized current pump pressure
        ])
        """

        # time at target is not used in the state
        self.state = np.array([
            self.current_angle / self.max_angle,  # Normalized current angle
            current_distance / self.max_angle,  # Normalized distance to target
            self.current_pressure / self.max_pressure  # Normalized current pump pressure
        ])



        return self.state

    def debug_print(self, reward,action):
        print("Debug Info:")
        print(f"Step: {self.current_time_step}")
        print(f"Current angle: {self.current_angle:.1f}")
        print(f"Target angle: {self.target_angle:.1f}")
        print(f"Angle difference: {(self.current_angle - self.target_angle):.1f}")
        print(f"Reward: {reward:.3f}")
        print(f"Servo input: {action:.1f}")
        print(f"Time at target: {self.time_at_target}")
        print("-" * 40)

    def set_debug_mode(self, mode: bool):
        self.debug_mode = mode

class Agent:
    def __init__(self):
        self.n_games = 0
        self.epsilon = 0
        self.gamma = 0.99
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet(3, 256, 1)  # Input: current_angle, angle_difference, (time_at_target removed), current_pressure; Output: action
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)

    def get_state(self, env):
        return env.get_state()

    def prefill_memory(self, data):
        for experience in data:
            self.remember(*experience)
        print(f"Prefilled memory with {len(data)} experiences from PID data.")

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train_long_memory(self):

        #print(f"Training long memory, memory len: {len(self.memory)}")

        if len(self.memory) > BATCH_SIZE:
            mini_sample = random.sample(self.memory, BATCH_SIZE)
        else:
            mini_sample = self.memory

        states, actions, rewards, next_states, dones = zip(*mini_sample)
        self.trainer.train_step(states, actions, rewards, next_states, dones)

    def train_short_memory(self, state, action, reward, next_state, done):
        self.trainer.train_step(state, action, reward, next_state, done)

    def get_action(self, state):
        # self.epsilon = 80 - self.n_games
        self.epsilon = max(0.1, 100 - self.n_games * 0.1)  # Decays more slowly and never goes below 0.1
        if random.randint(0, 200) < self.epsilon:
            return random.uniform(-1, 1)
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model(state0)
            return torch.tanh(prediction).item()  # Use tanh to bound the output between -1 and 1

    def _epsilon_greedy_action(self, state):
        self.epsilon = max(0.1, 100 - self.n_games * 0.1)  # Slower decay, minimum of 0.1
        if random.random() < self.epsilon:
            return random.uniform(-1, 1)
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model(state0)
            action = torch.tanh(prediction).item()
            noise = np.random.normal(0, 0.1)  # Add some noise for additional exploration
            return np.clip(action + noise, -1, 1)

    def _boltzmann_action(self, state):
        # cannot be used with single value output
        state0 = torch.tensor(state, dtype=torch.float)
        prediction = self.model(state0)

        # Ensure prediction is 1D
        if len(prediction.shape) > 1:
            prediction = prediction.squeeze()

        # If prediction is a single value, we can't use Boltzmann exploration
        if prediction.numel() == 1:
            return torch.tanh(prediction).item()

        temperature = max(0.1, 1.0 - self.n_games * 0.01)  # Decay temperature over time
        probabilities = F.softmax(prediction / temperature, dim=0)

        # Use torch.multinomial to sample an action
        action_index = torch.multinomial(probabilities, 1).item()

        # Scale the action to be between -1 and 1
        scaled_action = (action_index / (prediction.numel() - 1)) * 2 - 1
        return scaled_action

    def set_exploration_type(self, exploration_type):
        if exploration_type in ["epsilon-greedy", "boltzmann"]:
            self.exploration_type = exploration_type
        else:
            raise ValueError("Invalid exploration type. Choose 'epsilon-greedy' or 'boltzmann'.")

    def load(self, file_name='model.pth'):
        self.model.load_state_dict(torch.load(file_name))

    def save(self, file_name='model.pth'):
        torch.save(self.model.state_dict(), file_name)

def initialize_adc():
    adc_config_file = 'sensor_config.yaml'
    return ADC_hat(config_file=adc_config_file, simulation_mode=True)

def initialize_pwm():
    pwm_config_file = 'test_bench_config_RL.yaml'
    return PWM_hat(deadzone=0, simulation_mode=True, inputs=2, input_rate_threshold=0, config_file=pwm_config_file)

def calibrate_arm(sensor, pwm_driver):
    print("Starting calibration...")
    sensor.reset_angle_range()

    # Drive boom up
    pwm_driver.update_values([0.070, 1])
    time.sleep(1)  # Wait a bit to let pressure stabilize

    while True:

        current_angle, current_pressure = read_sensors(sensor)


        print(
            f"[UP] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")

        if current_pressure >= 200:
            pwm_driver.update_values([0.070, 0])  # Stop
            break
        time.sleep(0.1)

    # Drive boom down
    pwm_driver.update_values([0.070, -1])
    time.sleep(1) # Wait a bit to let pressure stabilize

    while True:
        current_angle, current_pressure = read_sensors(sensor)

        print(
            f"[DOWN] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")

        if current_pressure >= 200:
            pwm_driver.update_values([0.070, 0])  # Stop
            break
        time.sleep(0.1)

    # Move to midpoint
    ranges = sensor.get_angle_range()


    min_angle, max_angle = ranges["LiftBoom angle"]

    print(f"liftboom angles: {min_angle}, {max_angle}")

    mid_angle = (min_angle + (max_angle - min_angle) * 0.5)

    print(f"Moving to mid point: {mid_angle}")

    while True:
        current_angle, current_pressure = read_sensors(sensor)

        if abs(current_angle - mid_angle) < 1:  # 1 degree tolerance
            pwm_driver.update_values([-1.0, 0.0])  # Stop
            print(f"Arm positioned at middle angle: {current_angle:.1f}")
            print(f"Calibration complete! Arm range {min_angle}-{max_angle}.\n\n\n")
            time.sleep(1)
            break
        elif current_angle < mid_angle:
            # go down
            pwm_driver.update_values([0.070, -1.0])
        else:
            # go up
            pwm_driver.update_values([0.070, 1.0])
        time.sleep(0.1)

    return min_angle, max_angle

def read_sensors(sensor):
    # simple read function for testing purposes

    # Read the LiftBoom angle
    angles = sensor.read_filtered(read="angle")
    current_angle = angles["LiftBoom angle"]

    # Read the current pump pressure
    pressures = sensor.read_filtered(read="pressure")
    current_pressure = pressures["Pump"]

    return current_angle, current_pressure

def train(env, agent, debug=True):
    # Set debug mode for the environment
    env.set_debug_mode(debug)

    scores = []
    mean_scores = []
    total_score = 0

    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()  # Create a figure and axis object

    while True:
        state = env.reset()
        episode_score = 0
        done = False

        while not done:
            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            episode_score += reward

            agent.train_short_memory(state, action, reward, next_state, done)
            agent.remember(state, action, reward, next_state, done)

            state = next_state

        agent.n_games += 1
        agent.train_long_memory()

        scores.append(episode_score)
        total_score += episode_score
        mean_score = total_score / agent.n_games
        mean_scores.append(mean_score)

        ax.clear()  # Clear the axis for new data
        ax.plot(scores, 'b-', label='Score')
        ax.plot(mean_scores, 'r-', label='Average Score')
        ax.set_xlabel('Number of Games')
        ax.set_ylabel('Score')
        ax.set_title('Training Progress')
        ax.legend()

        if len(scores) > 0:
            ax.text(len(scores) - 1, scores[-1], str(round(scores[-1], 2)))
        if len(mean_scores) > 0:
            ax.text(len(mean_scores) - 1, mean_scores[-1], str(round(mean_scores[-1], 2)))

        plt.draw()
        plt.pause(0.001)  # Add a small pause to allow the plot to update

        # Early stopping condition
        if mean_score > 10000:
            print(f"Solved in {agent.n_games} episodes!")
            break

        # save every 50 games
        if agent.n_games % 50 == 0:
            agent.save('model_rl.pth')

    # ghetto hz limit, make better!
    time.sleep(0.1)
    return scores, mean_scores





# WIP stuff
def run_rl_without_training(env, agent, num_episodes=100):
    for episode in range(num_episodes):
        state = env.reset()
        done = False
        total_reward = 0

        while not done:
            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            total_reward += reward
            state = next_state

        print(f"Episode {episode + 1}, Total Reward: {total_reward}")

def load_pid_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            current_angle, target_angle, action, reward = map(float, row)
            state = np.array([
                current_angle / env.max_angle,
                (current_angle - target_angle) / env.max_angle,
                0,  # time_at_target, set to 0 as we don't have this info
                0   # current_pressure, set to 0 as we don't have this info
            ])
            next_state = state.copy()  # Assume next state is same as current for simplicity
            data.append((state, action, reward, next_state, False))
    return data


def run_pid(env, pid_controller, num_episodes=100, save_data=True):
    data_dir = 'pid_data'
    csv_file = os.path.join(data_dir, 'pid_training_data.csv')

    if save_data:
        os.makedirs(data_dir, exist_ok=True)
        csv_header = ['current_angle', 'target_angle', 'action', 'reward']

        # Check if file exists, if not create it and write header
        if not os.path.isfile(csv_file):
            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(csv_header)

    for episode in range(num_episodes):
        state = env.reset()
        done = False
        total_reward = 0

        while not done:
            state = env.update_state()
            current_angle = state[0] * env.max_angle

            pid_controller.SetPoint = env.target_angle
            pid_controller.update(current_angle)
            action = pid_controller.output

            # Clamp the output to [-1, 1]
            action = max(-1, min(1, action))

            # Flip output, as angle rises when servo goes down
            action = -action

            print(
                f"[PID] Current: {current_angle:.2f}, Target: {env.target_angle:.2f}, Error: {pid_controller.last_error:.2f}, Output: {action:.2f}")

            next_state, reward, done = env.step(action)
            total_reward += reward

            if save_data:
                with open(csv_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([current_angle, env.target_angle, action, reward])

            state = next_state

        print(f"Episode {episode + 1}, Total Reward: {total_reward}")

    if save_data:
        print(f"PID data saved to {csv_file}")


def interactive_mode(env, agent, pid_controller):
    current_mode = "RL"
    state = env.reset()

    print("Interactive Mode. Press 'q' to quit, 'r' for RL, 'p' for PID.")

    while True:
        if current_mode == "RL":
            action = agent.get_action(state)
        else:  # PID mode
            current_angle = state[0] * env.max_angle
            pid_controller.SetPoint = env.target_angle
            pid_controller.update(current_angle)
            action = pid_controller.output
            action = max(-1, min(1, action))  # Clamp to [-1, 1]
            action = -action  # Flip output

        next_state, reward, done = env.step(action)

        print(f"Mode: {current_mode}, Action: {action:.2f}, Reward: {reward:.2f}")

        if done:
            state = env.reset()
        else:
            state = next_state

        # Non-blocking input check
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            user_input = sys.stdin.readline().strip().lower()
            if user_input == 'q':
                break
            elif user_input == 'r':
                current_mode = "RL"
            elif user_input == 'p':
                current_mode = "PID"

def run_demo(env, agent, pid_controller):
    communicator = XRCommunicator(env)
    communicator.start()

    try:
        while True:
            data = communicator.get_latest_data()

            # Update target angle based on received data
            env.target_angle = data['target_angle']

            # Choose between RL and PID based on control mode
            if data['control_mode'] == 'RL':
                state = env.update_state()
                action = agent.get_action(state)
            else:
                current_angle = env.current_angle
                pid_controller.SetPoint = env.target_angle
                pid_controller.update(current_angle)
                action = pid_controller.output
                action = max(-1, min(1, action))  # Clamp to [-1, 1]
                action = -action  # Flip output

            # Apply the action
            next_state, reward, done = env.step(action)

            # Update the pump control value in the data
            data['pump_control_value'] = env.pwm.values[0]

            time.sleep(1 / LOOP_HZ)  # Maintain the loop frequency

    except KeyboardInterrupt:
        print("Demo stopped.")
    finally:
        communicator.stop()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run RL agent or PID controller')
    parser.add_argument('--mode', choices=['demo'], default='demo', help='Mode to run the script in')
    parser.add_argument('--checkpoint', type=str, help='Path to the checkpoint file to load')
    args = parser.parse_args()

    # Initialize the ADC and PWM
    sensor = initialize_adc()
    pwm_driver = initialize_pwm()

    min_angle, max_angle = calibrate_arm(sensor, pwm_driver)

    # Initialize the environment with the sensor and pwm_driver
    env = TestBenchEnv(sensor, pwm_driver, max_angle, min_angle)

    # Initialize the agent
    agent = Agent()

    # Load checkpoint if provided
    if args.checkpoint:
        if os.path.exists(args.checkpoint):
            agent.model.load_state_dict(torch.load(args.checkpoint))
            print(f"Loaded checkpoint from {args.checkpoint}")
        else:
            print(f"Checkpoint file {args.checkpoint} not found. Starting from scratch.")

    # Initialize PID controller
    pid_controller = PID(P=0.2, I=0.8, D=0.0)
    pid_controller.setSampleTime(1.0 / LOOP_HZ)
    pid_controller.setWindup(1.0)

    # Run the demo
    run_demo(env, agent, pid_controller)