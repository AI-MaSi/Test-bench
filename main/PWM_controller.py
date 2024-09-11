"""
This module implements a PWM (Pulse Width Modulation) controller for servo motors and other PWM-controlled devices.
It is designed to work with the Adafruit ServoKit library but can also run in a simulation mode.

Key features:
1. Configurable PWM channels using a YAML configuration file
2. Support for different types of PMW outputs: angle (servo), throttle, switch(WIP), and pump
3. Input rate monitoring to detect communication issues
4. Deadzone implementation to prevent unwanted small movements
5. Gamma correction for non-linear servo response
6. Simulation mode for testing without hardware

The main class, PWM_hat, handles:
- Initialization of PWM channels
- Parsing and validating configuration
- Updating PWM values based on input
- Handling special cases like pump control and track disabling
- Resetting the controller to a safe state
- Monitoring input rate for safety

Usage:
1. Create a YAML configuration file defining your PWM channels
2. Initialize the PWM_hat with the configuration file and desired settings
3. Call update_values() method with your input values to control the PWM outputs

"""


import threading
import yaml
import time

try:
    from adafruit_servokit import ServoKit
    SERVOKIT_AVAILABLE = True
except ImportError:
    SERVOKIT_AVAILABLE = False
    print("PWM module not found. Running in simulation mode.")


class PWM_hat:
    def __init__(self, inputs: int, config_file: str, simulation_mode: bool = False, pump_variable: bool = True,
                 tracks_disabled: bool = False, input_rate_threshold: float = 5, deadzone: int = 6) -> None:
        pwm_channels = 16
        print(f"PWM channels in use: {pwm_channels}")

        self.simulation_mode = simulation_mode

        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self.channel_configs = configs['CHANNEL_CONFIGS']

        self.pump_variable = pump_variable
        self.tracks_disabled = tracks_disabled

        self.values = [0.0 for _ in range(pwm_channels)]
        self.num_inputs = inputs
        self.num_outputs = pwm_channels

        self.input_rate_threshold = input_rate_threshold
        self.input_event = threading.Event()
        self.last_input_time = time.time()
        self.monitor_thread = None
        self.running = False

        self.center_val_servo = 90
        self.deadzone = deadzone

        self.return_servo_angles = False
        self.servo_angles = {}

        self.pump_enabled = True    # Enable pump by default
        self.pump_variable_sum = 0.0
        self.manual_pump_load = 0.0

        if SERVOKIT_AVAILABLE and not self.simulation_mode:
            self.kit = ServoKit(channels=pwm_channels)
        else:
            if not self.simulation_mode:
                print("ServoKit is not available. Falling back to simulation mode.")
            print("Using ServoKitStub for simulation.")
            self.kit = ServoKitStub(channels=pwm_channels)

        self.reset()
        self.validate_configuration()

        if input_rate_threshold > 0:    # Start monitoring if threshold is set
            self.start_monitoring()

    def validate_configuration(self) -> None:
        """Validate the configuration file."""
        required_keys = ['type', 'input_channel', 'output_channel', 'direction', 'offset']
        angle_specific_keys = ['multiplier_positive', 'multiplier_negative', 'gamma_positive', 'gamma_negative']

        for channel_name, config in self.channel_configs.items():
            # Validate existence of required keys
            for key in required_keys:
                if key not in config:
                    raise ValueError(f"Missing '{key}' in configuration for channel '{channel_name}'")

            # Validate types of operation
            if config['type'] not in ['angle', 'throttle', 'switch', 'none', 'pump']:
                raise ValueError(f"Invalid type '{config['type']}' for channel '{channel_name}'")

            # Validate input_channel
            if isinstance(config['input_channel'], int):
                if not (0 <= config['input_channel'] < self.num_inputs):
                    raise ValueError(
                        f"Input channel {config['input_channel']} out of range for channel '{channel_name}'")
            elif config['input_channel'] != 'none':
                raise ValueError(
                    f"Input channel must be an integer or 'none', got {type(config['input_channel'])} for channel '{channel_name}'")

            # Validate output_channel
            if isinstance(config['output_channel'], int):
                if not (0 <= config['output_channel'] < self.num_outputs):
                    raise ValueError(
                        f"Output channel {config['output_channel']} out of range for channel '{channel_name}'")
            else:
                raise ValueError(
                    f"Output channel must be an integer, got {type(config['output_channel'])} for channel '{channel_name}'")

            # Validate gamma values for angles
            if config['type'] == 'angle':
                for key in angle_specific_keys:
                    if key not in config:
                        raise ValueError(f"Missing '{key}' in configuration for angle type channel '{channel_name}'")
                    if 'gamma' in key:
                        if not (0.1 <= config[key] <= 3.0):
                            raise ValueError(
                                f"{key} {config[key]} out of range (0.1 to 3.0) for channel '{channel_name}'")
                    elif 'multiplier' in key:
                        if not (1 <= abs(config[key]) <= 50):
                            raise ValueError(f"{key} {config[key]} out of range (1 to 50) for channel '{channel_name}'")

            # Validate offset for reasonable adjustments
            if not (-30 <= config['offset'] <= 30):
                raise ValueError(f"Offset {config['offset']} out of range (-30 to 30) for channel '{channel_name}'")

        print("Validating configs done. Jee!")
        print("------------------------------------------\n")

    def start_monitoring(self) -> None:
        """Start the input rate monitoring thread."""
        if self.monitor_thread is None or not self.monitor_thread.is_alive():
            self.running = True
            self.monitor_thread = threading.Thread(target=self.monitor_input_rate, daemon=True)
            self.monitor_thread.start()
        else:
            print("Monitoring is already running.")

    def stop_monitoring(self) -> None:
        """Stop the input rate monitoring thread."""
        self.running = False
        if self.monitor_thread is not None:
            self.input_event.set()  # Wake up the thread if it's waiting
            self.monitor_thread.join()
            self.monitor_thread = None
            print("Stopped input rate monitoring...")

    def monitor_input_rate(self) -> None:
        """Monitor input rate using event-based approach."""
        print("Monitoring input rate...")
        while self.running:
            # Wait for an input event or timeout
            if self.input_event.wait(timeout=1.0 / self.input_rate_threshold):
                self.input_event.clear()
                self.last_input_time = time.time()
            else:
                # If we've timed out, check if we've exceeded our threshold
                if time.time() - self.last_input_time > 1.0 / self.input_rate_threshold:
                    print("Input rate too low. Resetting...")
                    self.reset(reset_pump=False)

    def update_values(self, raw_values, min_cap=-1, max_cap=1, return_servo_angles=False) :
        self.return_servo_angles = return_servo_angles
        self.servo_angles.clear()

        if raw_values is None:
            self.reset()
            raise ValueError("Input values are None")

        # Turn single input value to a list
        if isinstance(raw_values, float) or isinstance(raw_values, int):
            raw_values = [raw_values]

        if len(raw_values) != self.num_inputs:
            self.reset()
            raise ValueError(f"Expected {self.num_inputs} inputs, but received {len(raw_values)}.")

        deadzone_threshold = self.deadzone / 100.0 * (max_cap - min_cap)

        self.pump_variable_sum = 0.0
        for channel_name, config in self.channel_configs.items():
            input_channel = config['input_channel']
            if input_channel == 'none' or input_channel >= len(raw_values):
                continue

            # Check if the value is within the limits
            capped_value = max(min_cap, min(raw_values[input_channel], max_cap))

            # Check deadzone
            if abs(capped_value) < deadzone_threshold:
                capped_value = 0.0

            self.values[config['output_channel']] = capped_value

            if config.get('affects_pump', False):
                self.pump_variable_sum += abs(capped_value)

        self.handle_pump(self.values)
        self.handle_angles(self.values)
        # Signal the monitoring thread that we have new input
        self.input_event.set()

        if self.return_servo_angles:
            return self.servo_angles.copy()

    def handle_pump(self, values):
        pump_config = self.channel_configs['pump']
        pump_channel = pump_config['output_channel']
        pump_multiplier = pump_config['multiplier']
        pump_idle = pump_config['idle']
        input_channel = pump_config.get('input_channel', None)

        if not self.pump_enabled:
            throttle_value = -1.0  # Set to -1 when pump is disabled
        elif input_channel is None:
            if self.pump_variable:
                throttle_value = pump_idle + (pump_multiplier * self.pump_variable_sum)
            else:
                throttle_value = pump_idle + pump_multiplier

            # Add manual pump load
            throttle_value += self.manual_pump_load
        else:
            # pump_variable arg does not affect anything if input channel is set for the pump
            throttle_value = values[input_channel]

        throttle_value = max(-1.0, min(1.0, throttle_value))

        self.kit.continuous_servo[pump_channel].throttle = throttle_value

        return throttle_value  # Return the final throttle value for debugging

    def handle_angles(self, values):
        for channel_name, config in self.channel_configs.items():
            if config['type'] == 'angle':
                if self.tracks_disabled and channel_name in ['trackL', 'trackR']:
                    continue

                output_channel = config['output_channel']
                if output_channel >= len(values):
                    print(f"Channel '{channel_name}': No data available.")
                    continue

                input_value = values[output_channel]
                center = self.center_val_servo + config['offset']

                if input_value >= 0:
                    gamma = config.get('gamma_positive', 1)
                    multiplier = config.get('multiplier_positive', 1)
                    normalized_input = input_value
                else:
                    gamma = config.get('gamma_negative', 1)
                    multiplier = config.get('multiplier_negative', 1)
                    normalized_input = -input_value

                adjusted_input = normalized_input ** gamma
                gamma_corrected_value = adjusted_input if input_value >= 0 else -adjusted_input

                angle = center + (gamma_corrected_value * multiplier * config['direction'])
                angle = max(0, min(180, angle))

                self.kit.servo[config['output_channel']].angle = angle

                if self.return_servo_angles:
                    self.servo_angles[f"{channel_name} angle"] = round(angle, 2)

    def reset(self, reset_pump=True, pump_reset_point=-1.0):
        """
        Reset the controller to the initial state.

        :param reset_pump: Reset the pump to the idle state.
        :param pump_reset_point: The throttle value to set the pump to when resetting. ESC dependant.
        """

        for config in self.channel_configs.values():
            if config['type'] == 'angle':
                self.kit.servo[config['output_channel']].angle = self.center_val_servo + config.get('offset', 0)

        if reset_pump and 'pump' in self.channel_configs:
            self.kit.continuous_servo[self.channel_configs['pump']['output_channel']].throttle = pump_reset_point

    def set_threshold(self, number_value):
        """Update the input rate threshold value."""
        if not isinstance(number_value, (int, float)) or number_value <= 0:
            print("Threshold value must be a positive number.")
            return
        self.input_rate_threshold = number_value
        print(f"Threshold rate set to: {self.input_rate_threshold}Hz")

    def set_deadzone(self, int_value):
        """Update the Deadzone value"""
        if not isinstance(int_value, int):
            print("Deadzone value must be an integer.")
            return
        self.deadzone = int_value
        print(f"Deadzone set to: {self.deadzone}%")

    def set_tracks(self, bool_value):
        """Enable/Disable tracks"""
        if not isinstance(bool_value, bool):
            print("Tracks value value must be boolean.")
            return
        self.tracks_disabled = bool_value
        print(f"Tracks boolean set to: {self.tracks_disabled}!")

    def set_pump(self, bool_value):
        """Enable/Disable pump"""
        if not isinstance(bool_value, bool):
            print("Pump value must be boolean.")
            return
        self.pump_enabled = bool_value
        print(f"Pump enabled set to: {self.pump_enabled}!")
        # Update pump state immediately
        # self.handle_pump(self.values)

    def update_config(self, config_file):
        """Update the configuration file and reinitialize the controller."""
        # Reset the controller
        self.reset()

        # Re-read the config file
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self.channel_configs = configs['CHANNEL_CONFIGS']

        # Validate the new configuration
        self.validate_configuration()

        # Reinitialize necessary components
        if SERVOKIT_AVAILABLE and not self.simulation_mode:
            self.kit = ServoKit(channels=self.num_outputs)

        # Restart monitoring if it was running
        if self.running:
            self.stop_monitoring()
            self.start_monitoring()

        print(f"Configuration updated successfully from {config_file}")

    def print_input_mappings(self):
        """Print the input mappings for each channel."""
        print("Input mappings:")
        input_to_name = {}
        for channel_name, config in self.channel_configs.items():
            input_channel = config['input_channel']
            if input_channel != 'none' and isinstance(input_channel, int):
                if input_channel not in input_to_name:
                    input_to_name[input_channel] = []
                input_to_name[input_channel].append(channel_name)

        for input_num in range(self.num_inputs):
            if input_num in input_to_name:
                names = ', '.join(input_to_name[input_num])
                print(f"Input {input_num}: {names}")
            else:
                print(f"Input {input_num}: Not assigned")

    def update_pump(self, adjustment, debug=False):
        """
        Manually update the pump load.

        :param adjustment: The adjustment to add to the pump load (float between -1.0 and 1.0)
        :param debug: If True, print debug information
        """
        self.manual_pump_load = max(-1.0, min(1.0, self.manual_pump_load + adjustment))

        # Re-calculate pump throttle with new manual load
        current_throttle = self.handle_pump(self.values)

        if debug:
            print(f"Current pump throttle: {current_throttle:.2f}")
            print(f"Current manual pump load: {self.manual_pump_load:.2f}")
            print(f"Current pump variable sum: {self.pump_variable_sum:.2f}")

    def reset_pump_load(self, debug=False):
        """
        Reset the manual pump load to zero.

        :param debug: If True, print debug information
        """
        self.manual_pump_load = 0.0

        # Re-calculate pump throttle without manual load
        current_throttle = self.handle_pump(self.values)

        if debug:
            print(f"Pump load reset. Current pump throttle: {current_throttle:.2f}")

class ServoKitStub:
    def __init__(self, channels):
        self.channels = channels
        self.servo = [ServoStub() for _ in range(channels)]
        self.continuous_servo = [ContinuousServoStub() for _ in range(channels)]

class ServoStub:
    def __init__(self):
        self._angle = 90

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        self._angle = max(0, min(180, value))
        print(f"Servo angle set to: {self._angle}")

class ContinuousServoStub:
    def __init__(self):
        self._throttle = 0

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = max(-1, min(1, value))
        print(f"Continuous servo throttle set to: {self._throttle}")

