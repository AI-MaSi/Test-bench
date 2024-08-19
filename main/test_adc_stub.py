import ADC_sensors
import time
import matplotlib.pyplot as plt

# Initialize ADC sensors in simulation mode
sensors = ADC_sensors.ADC_hat(config_file='sensor_config.yaml', simulation_mode=True, frequency=0.2)

# List available sensors
sensors.list_sensors()
time.sleep(1)

# Initialize lists to store raw and filtered data
time_data = []
raw_data = {'Pressure': [], 'Angle': []}
filtered_data = {'Pressure': [], 'Angle': []}

# Time interval between measurements
time_interval = 0.1
start_time = time.time()

# Set up the plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(figsize=(12, 8))

while True:
    # Get the current time
    current_time = time.time() - start_time
    time_data.append(current_time)

    # Read raw and filtered data
    raw_readings = sensors.read_raw()
    #print("Raw Readings:", raw_readings)

    # Read filtered data
    filtered_readings_pressure = sensors.read_filtered('pressure')
    #print("Filtered Readings Pressure:", filtered_readings_pressure)
    filtered_readings_angle = sensors.read_filtered('angle')
    #print("Filtered Readings Angle:", filtered_readings_angle)

    # Extract raw values and calculate averages
    raw_pressure = [raw_readings.get(sensor_config['name'], 0)
                    for sensor_name, sensor_config in sensors.pressure_sensor_configs.items()]
    raw_angle = [raw_readings.get(sensor_config['name'], 0)
                 for sensor_name, sensor_config in sensors.angle_sensor_configs.items()]

    # Calculate average raw values
    avg_raw_pressure = sum(raw_pressure) / len(raw_pressure) if raw_pressure else 0
    avg_raw_angle = sum(raw_angle) / len(raw_angle) if raw_angle else 0

    # Extract filtered values
    filtered_pressure = filtered_readings_pressure[0] if filtered_readings_pressure else 0
    filtered_angle = filtered_readings_angle[0] if filtered_readings_angle else 0

    # Append data
    raw_data['Pressure'].append(avg_raw_pressure)
    raw_data['Angle'].append(avg_raw_angle)
    filtered_data['Pressure'].append(filtered_pressure)
    filtered_data['Angle'].append(filtered_angle)

    # Clear and update the plot
    ax.clear()
    ax.plot(time_data, raw_data['Pressure'], label='Raw Pressure', color='blue')
    ax.plot(time_data, raw_data['Angle'], label='Raw Angle', color='green')
    ax.plot(time_data, filtered_data['Pressure'], label='Filtered Pressure', color='red')
    ax.plot(time_data, filtered_data['Angle'], label='Filtered Angle', color='orange')

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Value')
    ax.legend()
    ax.set_title('Sensor Data Plot')
    ax.grid(True)

    plt.pause(time_interval)

    # Sleep for the time interval
    time.sleep(time_interval)
