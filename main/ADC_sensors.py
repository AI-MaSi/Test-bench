import yaml
import math
import time
import random

try:
    from ADCPi import ADCPi
except ImportError:
    print("ADCPi module not found. Running in simulation mode.")

class ADC_hat:
    def __init__(self, config_file, decimals=2, simulation_mode=False, min_sim_voltage=0.5, max_sim_voltage=4.5, frequency=1.0):
        """
        Initialize ADC_hat with configuration from a YAML file.

        :param config_file: Path to the YAML configuration file.
        :param decimals: Number of decimal places for calibration values.
        :param simulation_mode: Whether to run in simulation mode.
        :param min_sim_voltage: Minimum voltage for the SIMULATED ADC.
        :param max_sim_voltage: Maximum voltage for the SIMULATED ADC.
        :param frequency: Frequency of the sine wave for simulation.
        """
        self.decimals = decimals
        self.simulation_mode = simulation_mode

        if simulation_mode:
            # Replace ADCPi with a stub for simulation
            global ADCPi
            ADCPi = lambda addr1, addr2, bit_rate: ADCPiStub(addr1, addr2, bit_rate, min_sim_voltage, max_sim_voltage, frequency)
            print("Running in simulation mode.")

        # Load configuration from YAML file
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self.pressure_sensors = configs.get('PRESSURE_SENSORS', {})
            self.angle_sensors = configs.get('ANGLE_SENSORS', {})
            self.i2c_addresses = configs['ADC_CONFIG']['i2c_addresses']
            self.pga_gain = configs['ADC_CONFIG']['pga_gain']
            self.bit_rate = configs['ADC_CONFIG']['bit_rate']
            self.conversion_mode = configs['ADC_CONFIG']['conversion_mode']
            self.filter_configs = configs.get('FILTER_CONFIG', {})

        # Initialize ADC objects and data storage
        self.adcs = {}
        self.initialized = False
        self.min_voltage = {}
        self.pressure_data = {}
        self.angle_data = {}
        self.calibrated_angle_data = {}
        self.min_angle = {}
        self.max_angle = {}
        self.min_pressure = {}
        self.max_pressure = {}

        self.initialize_adc()

    def initialize_adc(self):
        board_needs_initialization = {board_name: False for board_name in self.i2c_addresses.keys()}

        # Determine which boards need initialization
        for sensor_config in self.pressure_sensors.values():
            board_name = sensor_config['input'][0]
            if board_name in board_needs_initialization:
                board_needs_initialization[board_name] = True

        for sensor_config in self.angle_sensors.values():
            board_name = sensor_config['input'][0]
            if board_name in board_needs_initialization:
                board_needs_initialization[board_name] = True

        print(f"Boards needed to init: {board_needs_initialization}")

        for board_name, need_init in board_needs_initialization.items():
            if need_init:
                addr1, addr2 = self.i2c_addresses[board_name]
                try:
                    adc_instance = ADCPi(addr1, addr2, self.bit_rate)
                    adc_instance.set_conversion_mode(self.conversion_mode)
                    adc_instance.set_pga(self.pga_gain)
                    self.adcs[board_name] = adc_instance
                    print(f"Initialized {board_name} with addresses {hex(addr1)}, {hex(addr2)}")
                    print(f"PGA Gain: {self.pga_gain}, Bit Rate: {self.bit_rate} bits, Conversion Mode: {self.conversion_mode}")
                except OSError as e:
                    raise OSError(f"Failed to initialize ADCPi for {board_name}! Error: {e}")

        self.initialized = True

    def read_raw(self):
        """
        Read raw voltage from all sensors as specified in the configuration.
        """
        if not self.initialized:
            print("ADCPi not initialized!")
            return {}

        raw_readings = {}
        for sensor_name, sensor_config in {**self.pressure_sensors, **self.angle_sensors}.items():
            voltage = self._read_raw(sensor_config)
            if voltage is not None:
                raw_readings[sensor_name] = voltage

        return raw_readings

    def _update_sensor_range(self, sensor_type, sensor_name, value):
        """
        Update the minimum and maximum range for a sensor.

        :param sensor_type: 'pressure' or 'angle' indicating the type of sensor.
        :param sensor_name: Name of the sensor to update.
        :param value: The current value to compare.
        """
        if sensor_type == 'pressure':
            min_dict, max_dict = self.min_pressure, self.max_pressure
        elif sensor_type == 'angle':
            min_dict, max_dict = self.min_angle, self.max_angle
        else:
            print(f"Unknown sensor type: {sensor_type}")
            return

        if sensor_name not in min_dict or value < min_dict[sensor_name]:
            min_dict[sensor_name] = value
        if sensor_name not in max_dict or value > max_dict[sensor_name]:
            max_dict[sensor_name] = value

    def read_scaled(self, read=None):
        """
        Read and scale the sensor data.

        :param read: Optional; Specify 'pressure', 'angle', or None to read both types.
        :return: A dictionary of scaled sensor readings.
        """
        scaled_readings = {}

        if read is None or read == 'pressure':
            for sensor_name, sensor_config in self.pressure_sensors.items():
                voltage = self._read_raw(sensor_config)
                if voltage is not None:
                    scaled_value = self.calibrate_pressure(voltage, sensor_config)
                    scaled_readings[sensor_name] = scaled_value
                    # Update pressure range
                    self._update_sensor_range('pressure', sensor_name, scaled_value)

        if read is None or read == 'angle':
            for sensor_name, sensor_config in self.angle_sensors.items():
                voltage = self._read_raw(sensor_config)
                if voltage is not None:
                    scaled_value = self.calibrate_angle(voltage, sensor_config)
                    scaled_readings[sensor_name] = scaled_value
                    # Update angle range
                    self._update_sensor_range('angle', sensor_name, scaled_value)

        return scaled_readings

    def read_filtered(self, read=None):
        # Step 1: Read and scale the sensor data
        scaled_data = self.read_scaled(read)
        #print(f"Scaled data: {scaled_data}")

        # Step 2: Apply filtering to the scaled data
        filtered_data = {}
        for sensor_name, scaled_value in scaled_data.items():
            sensor_config = self.pressure_sensors.get(sensor_name) or self.angle_sensors.get(sensor_name)
            if sensor_config:
                # Apply the sensor-specific filter
                sensor_filter_type = sensor_config.get('filter', self.filter_configs.get('default_filter', 'kalman'))
                filtered_value = self._apply_filter([scaled_value], sensor_filter_type, sensor_name)[
                    0]  # Extract the filtered value

                # Determine whether this is an angle or pressure sensor
                if sensor_name in self.angle_sensors:
                    min_angle = self.min_angle.get(sensor_name)
                    max_angle = self.max_angle.get(sensor_name)

                    if max_angle > min_angle:
                        adjusted_value = (filtered_value - min_angle) / (max_angle - min_angle) * (
                                    max_angle - min_angle)
                    else:
                        adjusted_value = filtered_value

                    # Update the angle range with the adjusted value
                    self._update_sensor_range('angle', sensor_name, adjusted_value)
                    filtered_data[sensor_name] = adjusted_value
                    #print(f"Updated angle data for {sensor_name}: {adjusted_value}")
                else:
                    # For pressure sensors, just use the filtered value
                    filtered_data[sensor_name] = filtered_value
                    #print(f"Updated pressure data for {sensor_name}: {filtered_value}")

        #print(f"Filtered data: {filtered_data}")
        return filtered_data

    def _read_raw(self, sensor_config):
        """
        Read raw voltage based on the sensor configuration.

        :param sensor_config: Configuration dictionary for the sensor.
        :return: Raw voltage reading.
        """
        board_name = sensor_config['input'][0]
        channel = sensor_config['input'][1]
        adc_instance = self.adcs.get(board_name)
        if adc_instance:
            return adc_instance.read_voltage(channel)
        else:
            print(f"ADC instance for board {board_name} not found.")
            return None

    def calibrate_pressure(self, voltage, sensor_config):
        """
        Calibrate pressure value based on the voltage.

        :param voltage: Raw voltage reading.
        :param sensor_config: Configuration dictionary for the sensor.
        :return: Calibrated pressure value.
        """
        if voltage > 0.40:
            return round((1000 * (voltage - 0.5) / (4.5 - 0.5)) * sensor_config['calibration_value'], self.decimals)
        else:
            return 0

    def calibrate_angle(self, voltage, sensor_config):
        """
        Calibrate angle value based on the voltage.

        :param voltage: Raw voltage reading.
        :param sensor_config: Configuration dictionary for the sensor.
        :return: Calibrated angle value.
        """
        steps_per_revolution = sensor_config.get('steps_per_revolution', 360)
        angle = round((voltage - 0.5) * steps_per_revolution / (4.5 - 0.5), self.decimals)

        return angle

    def _apply_filter(self, data, filter_type, sensor_name=None):
        """
        Apply the specified filter to the data.

        :param data: List of data values to filter.
        :param filter_type: Type of filter to apply ('low_pass' or 'kalman').
        :param sensor_name: Name of the sensor for sensor-specific filter settings.
        :return: Filtered data.
        """
        filter_settings = self.filter_configs.get(filter_type, {})

        if sensor_name and sensor_name in self.filter_configs:
            sensor_filter_config = self.filter_configs.get(sensor_name, {})
            filter_settings.update(sensor_filter_config)

        #print(f"Applying {filter_type} filter for {sensor_name}: {filter_settings}")

        if filter_type == 'low_pass':
            alpha = filter_settings.get('alpha', 0.5)
            return self._low_pass_filter(data, alpha)
        elif filter_type == 'kalman':
            Q = filter_settings.get('Q', 1e-5)
            R = filter_settings.get('R', 1e-2)
            P = filter_settings.get('P', 1.0)
            x0 = filter_settings.get('x0', data[0] if data else 0)

            #print(f"Kalman filter settings for {sensor_name} -> Q: {Q}, R: {R}, P: {P}, x0: {x0}")

            return self._kalman_filter(data, Q, R, P, x0)
        else:
            print(f"Unknown filter type: {filter_type}. Returning raw data.")
            return data

    def get_angle_range(self):
        """
        Calculate and return the minimum and maximum observed angle for each sensor.
        The range is computed as max - min.
        """
        angle_ranges = {}

        for sensor_name in self.angle_sensors.keys():
            min_angle = self.min_angle.get(sensor_name)
            max_angle = self.max_angle.get(sensor_name)
            angle_ranges[sensor_name] = (min_angle, max_angle)
        return angle_ranges

    def reset_angle_range(self):
        """
        Reset the range of calibrated angles.
        """
        self.min_angle = {}
        self.max_angle = {}

    def get_pressure_range(self):
        ranges = {}
        for sensor_name in self.pressure_sensors.keys():
            min_pressure = self.min_pressure.get(sensor_name)
            max_pressure = self.max_pressure.get(sensor_name)
            ranges[sensor_name] = (min_pressure, max_pressure)
        return ranges

    def reset_pressure_range(self):
        """
        Reset the range of calibrated pressures.
        """
        self.min_pressure = {}
        self.max_pressure = {}

    def list_sensors(self):
        if not self.initialized:
            print("ADCPi not initialized!")
            return

        print("Available Sensors:")
        print("Pressure Sensors:")
        for sensor_name in self.pressure_sensors.keys():
            print(f"  {sensor_name}")

        print("Angle Sensors:")
        for sensor_name in self.angle_sensors.keys():
            print(f"  {sensor_name}")
        print("-"*30)

    @staticmethod
    def _low_pass_filter(data, alpha):
        """
        Apply a low-pass filter to the data.

        :param data: List of data values to filter.
        :param alpha: Smoothing factor for the filter.
        :return: Filtered data.
        """
        filtered_data = []
        prev_value = data[0] if data else 0
        for value in data:
            filtered_value = alpha * prev_value + (1 - alpha) * value
            filtered_data.append(filtered_value)
            prev_value = filtered_value
        return filtered_data

    @staticmethod
    def _kalman_filter(data, Q, R, P, x0):
        """
        Apply a Kalman filter to the data.

        :param data: List of data values to filter.
        :param Q: Process noise covariance.
        :param R: Measurement noise covariance.
        :param P: Estimate covariance.
        :param x0: Initial estimate.
        :return: Filtered data.
        """
        x = x0
        filtered_data = []
        for z in data:
            P = P + Q
            K = P / (P + R)
            x = x + K * (z - x)
            P = (1 - K) * P
            filtered_data.append(x)
        return filtered_data


class ADCPiStub:
    def __init__(self, addr1, addr2, bit_rate, min_voltage, max_voltage, frequency):
        """
        Initialize a stub for simulating ADCPi behavior.

        :param addr1: First I2C address.
        :param addr2: Second I2C address.
        :param bit_rate: Bit rate of the ADC.
        :param min_voltage: Minimum voltage for simulation.
        :param max_voltage: Maximum voltage for simulation.
        :param frequency: Frequency of the sine wave for simulation.
        """
        self.addr1 = addr1
        self.addr2 = addr2
        self.bit_rate = bit_rate
        self.conversion_mode = 0
        self.pga_gain = 1
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        self.frequency = frequency
        self.start_time = time.time()

        # Noise stuff
        self.white_noise_level = 0.05
        self.pink_noise_level = 0.02
        self.spike_probability = 0.01
        self.spike_magnitude = 0.2

    def set_conversion_mode(self, mode):
        self.conversion_mode = mode

    def set_pga(self, gain):
        self.pga_gain = gain

    def generate_noise(self):
        """
        Generate noise for simulation.

        :return: Random noise value.
        """
        white_noise = random.uniform(-self.white_noise_level, self.white_noise_level)
        pink_noise = self.pink_noise_level * (random.random() - 0.5) / 2
        noise = white_noise + pink_noise
        if random.random() < self.spike_probability:
            noise += random.choice([-1, 1]) * self.spike_magnitude
        return noise

    def generate_wave(self, elapsed_time):
        """
        Generate a sine wave for simulation.

        :param elapsed_time: Time elapsed since the start.
        :return: Simulated voltage value.
        """
        amplitude = (self.max_voltage - self.min_voltage) / 2
        mid_point = self.min_voltage + amplitude
        return mid_point + amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

    def read_voltage(self, channel, create_sine=True, add_noise=True):

        # TODO: Return same amount of values as there are sensors configured

        """
        Read simulated voltage value.

        :param channel: ADC channel (not used in simulation).
        :param add_noise: Whether to add noise to the value.
        :return: Simulated voltage value.
        """
        elapsed_time = time.time() - self.start_time

        # Sine wave
        if create_sine:
            voltage = self.generate_wave(elapsed_time)
        else:
            voltage = random.uniform(self.min_voltage, self.max_voltage)
        # Noise
        if add_noise:
            voltage += self.generate_noise()
        return round(max(min(voltage, self.max_voltage), self.min_voltage), 2)
