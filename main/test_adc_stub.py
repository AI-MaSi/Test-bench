import ADC_sensors
import time
import matplotlib.pyplot as plt

# Initialize ADC sensors in simulation mode
sensors = ADC_sensors.ADC_hat(config_file='sensor_config.yaml', simulation_mode=True, frequency=0.1)

# List available sensors
sensors.list_sensors()
time.sleep(1)

# Initialize lists to store data
time_data = []
scaled_data = {}
filtered_data = {}

# Initialize data structures for each angle sensor
for sensor_name in sensors.angle_sensor_configs.keys():
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

    # Read scaled data
    scaled_readings = sensors.read_scaled()

    # Read filtered data for angle sensors
    filtered_readings_angle = sensors.read_filtered('angle', return_names=True, filter_type='low_pass')

    # Update data for each angle sensor
    for sensor_name, _, value in filtered_readings_angle:
        scaled_data[sensor_name].append(scaled_readings.get(sensor_name, 0))
        filtered_data[sensor_name].append(value)

    # Clear and update the plot
    ax.clear()

    # Plot angle data
    for sensor_name in sensors.angle_sensor_configs.keys():
        ax.plot(time_data, scaled_data[sensor_name], label=f'Scaled {sensor_name}', linestyle='--')
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