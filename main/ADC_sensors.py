import yaml
import math
import time
import random

try:
    from ADCPi import ADCPi
except ImportError:
    print("ADCPi module not found. Running in simulation mode.")

class ADC_hat:
    def __init__(self, config_file, decimals=2, simulation_mode=False, min_voltage=0.5, max_voltage=4.5, frequency=1.0):
        """
        Initialize ADC_hat with configuration from a YAML file.

        :param config_file: Path to the YAML configuration file.
        :param decimals: Number of decimal places for calibration values.
        :param simulation_mode: Whether to run in simulation mode.
        :param min_voltage: Minimum voltage for the simulated ADC.
        :param max_voltage: Maximum voltage for the simulated ADC.
        :param frequency: Frequency of the sine wave for simulation.
        """
        self.decimals = decimals
        self.simulation_mode = simulation_mode

        if simulation_mode:
            # Replace ADCPi with a stub for simulation
            global ADCPi
            ADCPi = lambda addr1, addr2, bit_rate: ADCPiStub(addr1, addr2, bit_rate, min_voltage, max_voltage, frequency)
            print("Running in simulation mode.")

        # Load configuration from YAML file
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self.pressure_sensor_configs = configs.get('PRESSURE_SENSORS', {})
            self.angle_sensor_configs = configs.get('ANGLE_SENSORS', {})
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
        self.min_calibrated_angle = {}
        self.max_calibrated_angle = {}
        self.min_calibrated_pressure = {}
        self.max_calibrated_pressure = {}

        self.initialize_adc()

    def initialize_adc(self):
        """
        Initialize ADC instances for boards based on their I2C addresses.
        """
        board_needs_initialization = {board_name: False for board_name in self.i2c_addresses.keys()}

        # Determine which boards need initialization
        for sensor_config in self.pressure_sensor_configs.values():
            board_name = sensor_config['input'][0]
            if board_name in board_needs_initialization:
                board_needs_initialization[board_name] = True

        for sensor_config in self.angle_sensor_configs.values():
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
        for sensor_config in self.pressure_sensor_configs.values():
            name = sensor_config['name']
            voltage = self._read_raw_internal(sensor_config)
            if voltage is not None:
                raw_readings[name] = voltage

        for sensor_config in self.angle_sensor_configs.values():
            name = sensor_config['name']
            voltage = self._read_raw_internal(sensor_config)
            if voltage is not None:
                raw_readings[name] = voltage

        return raw_readings

    def read_scaled(self):
        """
        Read and scale the sensor data.
        """
        if not self.initialized:
            print("ADCPi not initialized!")
            return {}

        scaled_readings = {}
        for sensor_name, sensor_config in self.pressure_sensor_configs.items():
            voltage = self._read_raw_internal(sensor_config)
            if voltage is not None:
                scaled_value = self.calibrate_pressure(voltage, sensor_config)
                scaled_readings[sensor_name] = scaled_value

        for sensor_name, sensor_config in self.angle_sensor_configs.items():
            voltage = self._read_raw_internal(sensor_config)
            if voltage is not None:
                scaled_value = self.calibrate_angle(voltage, sensor_config)
                scaled_readings[sensor_name] = scaled_value

        return scaled_readings

    def _read_raw_internal(self, sensor_config):
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

        if self.min_voltage.get(sensor_config['name']) is None or voltage < self.min_voltage[sensor_config['name']]:
            self.min_voltage[sensor_config['name']] = max(voltage, 0)

        calibrated_angle = round(
            angle - (self.min_voltage[sensor_config['name']] - 0.5) * steps_per_revolution / (4.5 - 0.5),
            self.decimals)

        return calibrated_angle

    def _apply_filter(self, data, filter_type=None, sensor_name=None):
        """
        Apply the specified filter to the data.

        :param data: List of data values to filter.
        :param filter_type: Type of filter to apply ('low_pass' or 'kalman').
        :param sensor_name: Name of the sensor for sensor-specific filter settings.
        :return: Filtered data.
        """
        if filter_type is None:
            filter_type = self.filter_configs.get('default_filter', 'kalman')

        filter_settings = self.filter_configs.get(filter_type, {})

        if sensor_name and sensor_name in self.filter_configs:
            sensor_filter_config = self.filter_configs[sensor_name]
            filter_settings.update(sensor_filter_config)

        if filter_type == 'low_pass':
            alpha = filter_settings.get('alpha', 0.5)
            return self._low_pass_filter(data, alpha)
        elif filter_type == 'kalman':
            Q = filter_settings.get('Q', 1e-5)
            R = filter_settings.get('R', 1e-2)
            P = filter_settings.get('P', 1.0)
            x0 = filter_settings.get('x0', data[0] if data else 0)
            return self._kalman_filter(data, Q, R, P, x0)
        else:
            print(f"Unknown filter type: {filter_type}. Returning raw data.")
            return data

    def read_filtered(self, sensor_type, filter_type='low_pass', return_names=False):
        """
        Read, scale, and filter the sensor data.

        :param sensor_type: Type of sensor data to read ('pressure' or 'angle').
        :param filter_type: Type of filter to apply ('low_pass' or 'kalman').
        :param return_names: Whether to return sensor names with values.
        :return: List of filtered sensor values or tuples of (name, description, value).
        """
        if not self.initialized:
            print("ADCPi not initialized!")
            return None

        scaled_data = self.read_scaled()

        if sensor_type == 'pressure':
            sensor_configs = self.pressure_sensor_configs
            data_store = self.pressure_data
        elif sensor_type == 'angle':
            sensor_configs = self.angle_sensor_configs
            data_store = self.angle_data
        else:
            print(f"Unknown sensor type: {sensor_type}.")
            return None

        if not sensor_configs:
            print(f"No {sensor_type} sensors configured.")
            return []

        sensor_data = []
        for sensor_name, sensor_config in sensor_configs.items():
            scaled_value = scaled_data.get(sensor_name)

            if scaled_value is None:
                filtered_value = None
            else:
                if sensor_name not in data_store:
                    data_store[sensor_name] = []
                data_store[sensor_name].append(scaled_value)

                filtered_value = self._apply_filter(data_store[sensor_name], filter_type, sensor_name)[-1]

                # Update min and max calibrated values
                if sensor_type == 'angle':
                    if sensor_name not in self.min_calibrated_angle or filtered_value < self.min_calibrated_angle[sensor_name]:
                        self.min_calibrated_angle[sensor_name] = filtered_value
                    if sensor_name not in self.max_calibrated_angle or filtered_value > self.max_calibrated_angle[sensor_name]:
                        self.max_calibrated_angle[sensor_name] = filtered_value
                elif sensor_type == 'pressure':
                    if sensor_name not in self.min_calibrated_pressure or filtered_value < self.min_calibrated_pressure[sensor_name]:
                        self.min_calibrated_pressure[sensor_name] = filtered_value
                    if sensor_name not in self.max_calibrated_pressure or filtered_value > self.max_calibrated_pressure[sensor_name]:
                        self.max_calibrated_pressure[sensor_name] = filtered_value

            if return_names:
                sensor_data.append((sensor_name, sensor_config.get('name', ''), filtered_value))
            else:
                sensor_data.append(filtered_value)

        return sensor_data

    def get_angle_range(self):
        """
        Get the range of calibrated angles for each angle sensor.
        """
        ranges = {}
        for sensor_name in self.angle_sensor_configs.keys():
            min_angle = self.min_calibrated_angle.get(sensor_name, None)
            max_angle = self.max_calibrated_angle.get(sensor_name, None)
            ranges[sensor_name] = (min_angle, max_angle)
        return ranges

    def reset_angle_range(self):
        """
        Reset the range of calibrated angles.
        """
        self.min_calibrated_angle = {}
        self.max_calibrated_angle = {}

    def get_pressure_range(self):
        """
        Get the range of calibrated pressures for each pressure sensor.
        """
        ranges = {}
        for sensor_name in self.pressure_sensor_configs.keys():
            min_pressure = self.min_calibrated_pressure.get(sensor_name, None)
            max_pressure = self.max_calibrated_pressure.get(sensor_name, None)
            ranges[sensor_name] = (min_pressure, max_pressure)
        return ranges

    def reset_pressure_range(self):
        """
        Reset the range of calibrated pressures.
        """
        self.min_calibrated_pressure = {}
        self.max_calibrated_pressure = {}

    def list_sensors(self):
        """
        List all available and initialized sensors.
        """
        if not self.initialized:
            print("ADCPi not initialized!")
            return

        print("Available Sensors:")
        print("Pressure Sensors:")
        for sensor_name in self.pressure_sensor_configs.keys():
            print(f"  {sensor_name}")

        print("Angle Sensors:")
        for sensor_name in self.angle_sensor_configs.keys():
            print(f"  {sensor_name}")

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
