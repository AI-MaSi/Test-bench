import ADC_sensors
import time
import matplotlib.pyplot as plt


#import joystick
#import PWM_controller

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

# controller
#joy = joystick.XboxController()

# Initialize ADC sensors in simulation mode
sensors = ADC_sensors.ADC_hat(config_file='sensor_config.yaml', simulation_mode=True, frequency=0.1)


"""
pwm = PWM_controller.PWM_hat(config_file='test_bench_config_RL.yaml',
                                  inputs=2,
                                  simulation_mode=True,
                                  pump_variable=False,
                                  tracks_disabled=True,
                                  input_rate_threshold=0,
                                  deadzone=0)
"""
# List available sensors
sensors.list_sensors()
time.sleep(1)

# Initialize lists to store data
time_data = []
scaled_data = {}
filtered_data = {}

# Initialize data structures for each angle sensor
for sensor_name in sensors.angle_sensors.keys():
    scaled_data[sensor_name] = []
    filtered_data[sensor_name] = []

# Time interval between measurements
time_interval = 0.02
start_time = time.time()

# Set up the plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(figsize=(12, 8))

while True:
    # Get the current time
    current_time = time.time() - start_time
    time_data.append(current_time)

    """
    values = joy.read()

    if values:
        value = values[1]
    else:
        value = 0

    print(value)

    # control servo
    #pwm.update_values([0.070, value])
    """

    # Read scaled data. Returns a dictionary with sensor names as keys. All the sensors!
    scaled_readings = sensors.read_scaled()

    # Read scaled and filtered angle data
    angle_readings = sensors.read_filtered('angle')

    # test. Read everything
    #angle_readings2 = sensors.read_filtered()
    #print(f"Read everything filtered: {angle_readings2}")
    #time.sleep(2)

    # Update data for each angle sensor. If the sensor is not available, the value will be 0
    for sensor_name, value in angle_readings.items():
        scaled_data[sensor_name].append(scaled_readings.get(sensor_name, 0))
        filtered_data[sensor_name].append(value)

    # Clear and update the plot
    ax.clear()

    # Plot angle data
    for sensor_name in sensors.angle_sensors.keys():
        ax.plot(time_data, scaled_data[sensor_name], label=f'Scaled (raw) {sensor_name}', linestyle='--')
        ax.plot(time_data, filtered_data[sensor_name], label=f'Filtered {sensor_name}')

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Angle')
    ax.legend()
    ax.set_title('Angle Sensor Data')
    ax.grid(True)

    plt.tight_layout()
    plt.pause(time_interval)

    # Sleep for the time interval
    time.sleep(time_interval)


