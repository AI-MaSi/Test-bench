import yaml
import math
import time

try:
    from ADCPi import ADCPi
except ImportError:
    print("ADCPi module not found. Running in simulation mode.")


class ADC_hat:
    def __init__(self, config_file, decimals=2, simulation_mode=False, min_voltage=0.5, max_voltage=4.5, frequency=1.0):
        self.decimals = decimals
        self.simulation_mode = simulation_mode

        if simulation_mode:
            # Replace ADCPi with the stub
            global ADCPi
            ADCPi = lambda addr1, addr2, bit_rate: ADCPiStub(addr1, addr2, bit_rate, min_voltage, max_voltage,
                                                             frequency)
            print("Running in simulation mode.")

        # Load configs from .yaml file
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self.pressure_sensor_configs = configs.get('PRESSURE_SENSORS', {})
            self.angle_sensor_configs = configs.get('ANGLE_SENSORS', {})

            # ADCPi settings
            self.i2c_addresses = configs['ADC_CONFIG']['i2c_addresses']
            self.pga_gain = configs['ADC_CONFIG']['pga_gain']
            self.bit_rate = configs['ADC_CONFIG']['bit_rate']
            self.conversion_mode = configs['ADC_CONFIG']['conversion_mode']

            # Load filter configurations
            self.filter_configs = configs.get('FILTER_CONFIGS', {})

        # Initialize a dictionary to hold ADC objects
        self.adcs = {}
        self.initialized = False
        self.min_voltage = {}

        # Initialize data storage
        self.pressure_data = {}
        self.angle_data = {}
        self.calibrated_angle_data = {}

        # Initialize min and max calibrated angles
        self.min_calibrated_angle = {}
        self.max_calibrated_angle = {}

        # Initialize min and max calibrated pressures
        self.min_calibrated_pressure = {}
        self.max_calibrated_pressure = {}

        # Initialize ADC instances
        self.initialize_adc()

    def initialize_adc(self):
        board_needs_initialization = {board_name: False for board_name in self.i2c_addresses.keys()}

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
                # hat uses two addresses
                addr1, addr2 = self.i2c_addresses[board_name]

                try:
                    # Create an instance of ADCPi with the addresses
                    adc_instance = ADCPi(addr1, addr2, self.bit_rate)
                    adc_instance.set_conversion_mode(self.conversion_mode)
                    adc_instance.set_pga(self.pga_gain)

                    # Store the initialized instance in the dictionary
                    self.adcs[board_name] = adc_instance

                    print(f"Initialized {board_name} with addresses {hex(addr1)}, {hex(addr2)}")
                    print(
                        f"PGA Gain: {self.pga_gain}, Bit Rate: {self.bit_rate} bits, Conversion Mode: {self.conversion_mode}")
                except OSError as e:
                    raise OSError(f"Failed to initialize ADCPi for {board_name}! Error: {e}")

        self.initialized = True

    def read_raw(self):
        """Read raw voltage from all sensors, using the names specified in the config."""
        if not self.initialized:
            print("ADCPi not initialized!")
            return {}

        raw_readings = {}

        # Read pressure sensors
        for sensor_config in self.pressure_sensor_configs.values():
            name = sensor_config['name']
            voltage = self._read_raw_internal(sensor_config)
            if voltage is not None:
                raw_readings[name] = voltage

        # Read angle sensors
        for sensor_config in self.angle_sensor_configs.values():
            name = sensor_config['name']
            voltage = self._read_raw_internal(sensor_config)
            if voltage is not None:
                raw_readings[name] = voltage

        return raw_readings

    def _read_raw_internal(self, sensor_config):
        """Internal method to read raw voltage based on sensor configuration."""
        board_name = sensor_config['input'][0]
        channel = sensor_config['input'][1]
        adc_instance = self.adcs.get(board_name)
        if adc_instance:
            return adc_instance.read_voltage(channel)
        else:
            print(f"ADC instance for board {board_name} not found.")
            return None

    def calibrate_pressure(self, voltage, sensor_config):
        """Calibrate the pressure value based on the voltage."""
        if voltage > 0.40:
            return round((1000 * (voltage - 0.5) / (4.5 - 0.5)) * sensor_config['calibration_value'], self.decimals)
        else:
            return 0

    def calibrate_angle(self, voltage, sensor_config):
        """Calibrate the angle value based on the voltage."""

        #  voltage threshold?

        steps_per_revolution = sensor_config.get('steps_per_revolution', 360)
        angle = round((voltage - 0.5) * steps_per_revolution / (4.5 - 0.5), self.decimals)

        if self.min_voltage.get(sensor_config['name']) is None or voltage < self.min_voltage[sensor_config['name']]:
            self.min_voltage[sensor_config['name']] = max(voltage, 0)  # remove negative values

        calibrated_angle = round(
            angle - (self.min_voltage[sensor_config['name']] - 0.5) * steps_per_revolution / (4.5 - 0.5),
            self.decimals)

        # angle removed for now
        # return angle, calibrated_angle
        return calibrated_angle

    def apply_filter(self, data, filter_type=None, sensor_name=None):
        """Apply the specified filter to the data."""
        # Use default filter type if none is provided
        if filter_type is None:
            filter_type = self.filter_configs.get('default_filter', 'low_pass')

        # Get filter settings
        filter_settings = self.filter_configs.get(filter_type, {})

        # Override default settings with sensor-specific settings if available
        if sensor_name and sensor_name in self.filter_configs:
            sensor_filter_config = self.filter_configs[sensor_name]
            filter_settings.update(sensor_filter_config)

        if filter_type == 'low_pass':
            alpha = filter_settings.get('alpha', 0.5)  # Default to 0.5 if not specified
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
        """Read and filter the sensor data."""
        if not self.initialized:
            print("ADCPi not initialized!")
            return None

        if sensor_type == 'pressure':
            sensor_configs = self.pressure_sensor_configs
            calibrate_func = self.calibrate_pressure
            data_store = self.pressure_data
        elif sensor_type == 'angle':
            sensor_configs = self.angle_sensor_configs
            calibrate_func = self.calibrate_angle
            data_store = self.angle_data
        else:
            print(f"Unknown sensor type: {sensor_type}.")
            return None

        if not sensor_configs:
            print(f"No {sensor_type} sensors configured.")
            return []

        sensor_data = []
        for sensor_name, sensor_config in sensor_configs.items():
            voltage = self._read_raw_internal(sensor_config)

            if voltage is None:
                filtered_value = None
            else:
                calibrated_angle = self.calibrate_angle(voltage, sensor_config)


                if sensor_name not in data_store:
                    data_store[sensor_name] = []
                data_store[sensor_name].append(calibrated_angle)

                filtered_value = self.apply_filter(data_store[sensor_name], filter_type, sensor_name)[-1]

                # Update min and max calibrated angles
                if sensor_name not in self.min_calibrated_angle or filtered_value < self.min_calibrated_angle[
                    sensor_name]:
                    self.min_calibrated_angle[sensor_name] = filtered_value
                if sensor_name not in self.max_calibrated_angle or filtered_value > self.max_calibrated_angle[
                    sensor_name]:
                    self.max_calibrated_angle[sensor_name] = filtered_value

            if return_names:
                sensor_data.append((sensor_name, sensor_config.get('name', ''), filtered_value))
            else:
                sensor_data.append(filtered_value)

        return sensor_data

    def get_angle_range(self):
        ranges = {}
        for sensor_name in self.angle_sensor_configs.keys():
            min_angle = self.min_calibrated_angle.get(sensor_name, None)
            max_angle = self.max_calibrated_angle.get(sensor_name, None)
            ranges[sensor_name] = (min_angle, max_angle)
        return ranges

    def reset_angle_range(self):
        self.min_calibrated_angle = {}
        self.max_calibrated_angle = {}

    def get_pressure_range(self):
        ranges = {}
        for sensor_name in self.pressure_sensor_configs.keys():
            min_pressure = self.min_calibrated_pressure.get(sensor_name, None)
            max_pressure = self.max_calibrated_pressure.get(sensor_name, None)
            ranges[sensor_name] = (min_pressure, max_pressure)
        return ranges

    def reset_pressure_range(self):
        self.min_calibrated_pressure = {}
        self.max_calibrated_pressure = {}

    def list_sensors(self):
        """List all available and initialized sensors."""
        if not self.initialized:
            print("ADCPi not initialized!")
            return

        print("Available Sensors:")
        print("Pressure Sensors:")
        for sensor_name, sensor_config in self.pressure_sensor_configs.items():
            print(f"  {sensor_name}")

        print("Angle Sensors:")
        for sensor_name, sensor_config in self.angle_sensor_configs.items():
            print(f"  {sensor_name}")

    @staticmethod
    def _low_pass_filter(data, alpha=1.0):
        filtered_data = []
        prev_value = data[0] if data else 0
        for value in data:
            filtered_value = alpha * prev_value + (1 - alpha) * value
            filtered_data.append(filtered_value)
            prev_value = filtered_value
        return filtered_data

    @staticmethod
    def _kalman_filter(data, Q, R, P, x0):
        x = x0
        filtered_data = []
        for z in data:
            # Prediction update
            P = P + Q

            # Measurement update
            K = P / (P + R)
            x = x + K * (z - x)
            P = (1 - K) * P

            filtered_data.append(x)
        return filtered_data


class ADCPiStub:
    """
    This is a stub class for the ADCPi module.
    It simulates the ADCPi module by generating random voltage readings.
    """
    def __init__(self, addr1, addr2, bit_rate, min_voltage, max_voltage, frequency):
        self.addr1 = addr1
        self.addr2 = addr2
        self.bit_rate = bit_rate
        self.conversion_mode = 0
        self.pga_gain = 1
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        self.frequency = frequency
        self.start_time = time.time()

    def set_conversion_mode(self, mode):
        self.conversion_mode = mode

    def set_pga(self, gain):
        self.pga_gain = gain

    def read_voltage(self, channel):
        # Calculate the time elapsed since the start
        elapsed_time = time.time() - self.start_time

        # Generate a sine wave between min_voltage and max_voltage
        amplitude = (self.max_voltage - self.min_voltage) / 2
        mid_point = self.min_voltage + amplitude

        # Frequency controls the speed of the wave
        voltage = mid_point + amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        return round(voltage, 2)