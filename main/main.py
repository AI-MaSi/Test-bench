# PID control testing file

import tkinter as tk
from tkinter import messagebox, scrolledtext
import masi_driver
import GPIO_sensors
import ADC_sensors
import yaml
import serial
import time
import threading
import serial.tools.list_ports

# Pump stuff
step = 0.005
min_cap = -0.1
max_cap = 0.2

# Initial (default) settings
Setpoint = -5.0
Kp = 5.0
Ki = 0.8
Kd = 1.0
servo_center = 90.0
angle_limit = 30.0
deadzone = 0.0  # useless, remove this!

# Load configuration
with open('test_bench_config.yaml', 'r') as file:
    config = yaml.safe_load(file)
    initial_pump_value = config['CHANNEL_CONFIGS']['pump'].get('idle', min_cap)  # Default to min_cap if not specified

# Initialize the pump driver
pump_driver = masi_driver.ExcavatorController(deadzone=0, simulation_mode=False, inputs=1, input_rate_threshold=1,
                                              config_file='pump_config.yaml')

# Initialize the RPM measurement
# make the measurement more accurate!
rpm_sensor = GPIO_sensors.RPMSensor(config_file='sensor_config.yaml', sensor_name='RPM_pump')

# pressure and angle measurement
adc_sensors = ADC_sensors.SensorManager(config_file='sensor_config.yaml')


def find_serial_device():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"found port: {port.description}")
        if 'XIAO RP2040' in port.description:
            return port.device
    return None


# Find and initialize serial communication
serial_device = find_serial_device()
if serial_device:
    ser = serial.Serial(serial_device, baudrate=115200, timeout=0.1)
    print(f"Using serial device: {serial_device}")
else:
    ser = None
    raise RuntimeError("No serial device found")


# Send initial values to serial device
def send_initial_values():
    initial_values = [
        0.0,  # Input placeholder
        0.0,  # Output placeholder
        Setpoint,
        Kp,
        Ki,
        Kd,
        servo_center,
        angle_limit,
        deadzone
    ]
    ser.write((','.join(map(str, initial_values)) + '\n').encode())


send_initial_values()


# Stop the ESC beep if first start
def reset_pump():
    value = -1.0
    delay = 1.0
    pump_driver.update_values(value, min_cap=min_cap)
    print(f"Sent reset command to pump ({value})")
    time.sleep(delay)
    print(f"Waited {delay} seconds after reset")


reset_pump()

# Initialize Tkinter
root = tk.Tk()
root.title('Pump and RPM Monitor')
root.geometry("1000x800")  # Make the window wider and taller

# Initial values
value = initial_pump_value
last_value = value
pump_stopped = False
stop_serial_thread = False

fields = [
    "Setpoint", "Kp", "Ki", "Kd", "servo_center", "angle_limit", "deadzone"
]

# User input limits for the GUI
limits = {
    'Setpoint': (-100.0, 100.0),
    'Kp': (0.0, 10.0),
    'Ki': (0.0, 10.0),
    'Kd': (0.0, 10.0),
    'servo_center': (0, 180),
    'angle_limit': (0, 100),
    'deadzone': (0, 1.0)
}

entries = {}
labels = {}
current_values = {}

frame = tk.Frame(root, bd=2, relief=tk.RIDGE)
frame.grid(row=0, column=0, rowspan=len(fields), padx=10, pady=10)
tk.Label(frame, text="Valve", font=('Arial', 14, 'bold')).grid(row=0, column=0, columnspan=2)

for i, field in enumerate(fields):
    labels[field] = tk.Label(frame, text=field)
    labels[field].grid(row=i + 1, column=0)
    entries[field] = tk.Entry(frame)
    entries[field].grid(row=i + 1, column=1)

    # Use the initial values defined in your code
    if field == 'Setpoint':
        initial_value = Setpoint
    elif field == 'Kp':
        initial_value = Kp
    elif field == 'Ki':
        initial_value = Ki
    elif field == 'Kd':
        initial_value = Kd
    elif field == 'servo_center':
        initial_value = servo_center
    elif field == 'angle_limit':
        initial_value = angle_limit
    elif field == 'deadzone':
        initial_value = deadzone

    entries[field].insert(0, str(initial_value))

    current_values[field] = tk.Label(frame, text="0.0")
    current_values[field].grid(row=i + 1, column=2)

# Function to read serial data
serial_data = {
    'Input': 0.0,
    'Output': 0.0,
    'Setpoint': 0.0,
    'Kp': 0.0,
    'Ki': 0.0,
    'Kd': 0.0,
    'servo_center': 0.0,
    'lim': 0.0,
    'deadzone': 0.0
}


def read_serial_data():
    while ser and ser.is_open and not stop_serial_thread:
        if ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            if message:
                print("Received:", message)
                try:
                    values = message.split(',')
                    if len(values) == 9:
                        serial_data['Input'] = float(values[0])
                        serial_data['Output'] = float(values[1])
                        serial_data['Setpoint'] = float(values[2])
                        serial_data['Kp'] = float(values[3])
                        serial_data['Ki'] = float(values[4])
                        serial_data['Kd'] = float(values[5])
                        serial_data['servo_center'] = float(values[6])
                        serial_data['lim'] = float(values[7])
                        serial_data['deadzone'] = float(values[8])
                except ValueError:
                    print("Error parsing serial data")


# Start thread for reading serial data
if ser:
    thread_read = threading.Thread(target=read_serial_data)
    thread_read.daemon = True
    thread_read.start()


# Function to update the current values in the GUI
def update_current_values():
    current_values['Input'].config(text=f"{serial_data['Input']:.1f}")
    current_values['Output'].config(text=f"{serial_data['Output']:.1f}")
    root.after(200, update_current_values)


def update_pump_value():
    global value, last_value, pump_stopped
    if not pump_stopped:
        value = max(min(value, max_cap), min_cap)
    pump_driver.update_values(value, min_cap=min_cap)
    rpm_label.config(text=f'RPM: {rpm_sensor.read_rpm():.0f}')
    pump_value_label.config(text=f'Pump Value: {value:.3f}')
    root.after(100, update_pump_value)


# Start and stop pump
def toggle_pump():
    global value, last_value, pump_stopped
    if pump_stopped:
        value = last_value
        pump_stopped = False
        start_stop_button.config(text="Stop Pump")
    else:
        last_value = value
        value = min_cap
        pump_stopped = True
        start_stop_button.config(text="Start Pump")


# Function to send values to the serial device
def update_values():
    new_values = [
        serial_data['Input'],
        serial_data['Output'],
        float(entries["Setpoint"].get()),
        float(entries["Kp"].get()),
        float(entries["Ki"].get()),
        float(entries["Kd"].get()),
        float(entries["servo_center"].get()),
        float(entries["angle_limit"].get()),
        float(entries["deadzone"].get())
    ]

    # Check limits
    for field, value in zip(fields, new_values[2:]):
        if value < limits[field][0] or value > limits[field][1]:
            messagebox.showerror("Invalid Input", f"{field} must be between {limits[field][0]} and {limits[field][1]}")
            return

    ser.write((','.join(map(str, new_values)) + '\n').encode())


# Function to fine-tune the pump value
def fine_tune_pump(amount):
    global value
    value = max(min(value + amount, max_cap), min_cap)


# Pump control elements
pump_frame = tk.Frame(root, bd=2, relief=tk.RIDGE)
pump_frame.grid(row=0, column=8, rowspan=8, padx=10, pady=10)

rpm_label = tk.Label(pump_frame, text=f'RPM: {rpm_sensor.read_rpm():.0f}')
rpm_label.grid(row=0, column=0, columnspan=2)

pump_value_label = tk.Label(pump_frame, text=f'Pump Value: {value:.3f}')
pump_value_label.grid(row=1, column=0, columnspan=2)

start_stop_button = tk.Button(pump_frame, text="Stop Pump", command=toggle_pump)
start_stop_button.grid(row=2, column=0, columnspan=2)

# Fine-tune buttons for pump value
fine_tune_frame = tk.Frame(pump_frame)
fine_tune_frame.grid(row=5, column=0, columnspan=2, pady=10)
tk.Button(fine_tune_frame, text="+ step", command=lambda: fine_tune_pump(step)).grid(row=0, column=0)
tk.Button(fine_tune_frame, text="- step", command=lambda: fine_tune_pump(-step)).grid(row=0, column=1)

# Create a frame to display Input and Output values
io_frame = tk.Frame(root, bd=2, relief=tk.RIDGE)
io_frame.grid(row=8, column=8, rowspan=8, padx=10, pady=10)

tk.Label(io_frame, text="Input (KalY):").grid(row=0, column=0)
tk.Label(io_frame, text="Output1 (Servo Angle):").grid(row=2, column=0)

current_values['Input'] = tk.Label(io_frame, text="0.0")
current_values['Input'].grid(row=0, column=1)
current_values['Output'] = tk.Label(io_frame, text="0.0")
current_values['Output'].grid(row=2, column=1)

# Modify the existing debug box
# Replace the debug box with a box for pressure sensor readings
pressure_frame = tk.Frame(root, bd=2, relief=tk.RIDGE)
pressure_frame.grid(row=16, column=0, columnspan=9, padx=10, pady=5)

tk.Label(pressure_frame, text="Pressure Sensors", font=('Arial', 12, 'bold')).grid(row=0, column=0, columnspan=2)

# Add labels for each pressure sensor (assuming 2 sensors for this example)
pressure_labels = []
for i in range(2):
    label = tk.Label(pressure_frame, text=f"Pressure Sensor {i + 1}: 0.0")
    label.grid(row=i + 1, column=0, columnspan=2)
    pressure_labels.append(label)

# Add a new box for angle sensor1
angle_frame = tk.Frame(root, bd=2, relief=tk.RIDGE)
angle_frame.grid(row=17, column=0, columnspan=9, padx=10, pady=5)

tk.Label(angle_frame, text="Angle Sensor 1", font=('Arial', 12, 'bold')).grid(row=0, column=0, columnspan=2)
raw_angle_label = tk.Label(angle_frame, text="Raw: ", font=('Arial', 10))
raw_angle_label.grid(row=1, column=0)
calibrated_angle_label = tk.Label(angle_frame, text="Calibrated: ", font=('Arial', 10))
calibrated_angle_label.grid(row=1, column=1)
angular_velocity_label = tk.Label(angle_frame, text="Angular Velocity: ", font=('Arial', 10))
angular_velocity_label.grid(row=2, column=0, columnspan=2)

# Variables to store previous angle and time for angular velocity calculation
prev_angle = None
prev_time = None


# Function to update pressure sensor information
def update_sensor_info():
    global prev_angle, prev_time

    # Get angle readings
    angle_readings = adc_sensors.read_angle(return_names=False)

    # Get pressure readings
    pressure_readings = adc_sensors.read_pressure(return_names=False)


    # Update pressure sensor labels
    for i, value in enumerate(pressure_readings):
        if i < len(pressure_labels):
            pressure_labels[i].config(text=f"Pressure Sensor {i + 1}: {value:.1f}")

    if angle_readings and len(angle_readings) > 0:
        # Update angle sensor1 information
        raw_angle, calibrated_angle = angle_readings[0]  # Assuming the first value is from angle sensor1

        raw_angle_label.config(text=f"Raw: {raw_angle:.1f}°")
        calibrated_angle_label.config(text=f"Calibrated: {calibrated_angle:.1f}°")

        # Calculate angular velocity
        current_time = time.time()
        if prev_angle is not None and prev_time is not None:
            time_diff = current_time - prev_time
            angle_diff = calibrated_angle - prev_angle
            angular_velocity = angle_diff / time_diff
            angular_velocity_label.config(text=f"Angular Velocity: {angular_velocity:.1f}°/s")
        else:
            angular_velocity_label.config(text="Angular Velocity: N/A")

        # Update previous values
        prev_angle = calibrated_angle
        prev_time = current_time
    else:
        raw_angle_label.config(text="Raw: N/A")
        calibrated_angle_label.config(text="Calibrated: N/A")
        angular_velocity_label.config(text="Angular Velocity: N/A")

    root.after(100, update_sensor_info)  # Update every 100ms


# Add update button
update_button = tk.Button(root, text="Update", command=update_values)
update_button.grid(row=len(fields) + 1, padx=10, pady=10)

# Start the main loop and the update loops
update_pump_value()
update_current_values()
update_sensor_info()
root.mainloop()

# Stop the serial thread
stop_serial_thread = True

if ser:
    ser.close()
