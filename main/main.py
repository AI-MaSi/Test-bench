import pygame
import masi_driver
import sensor_manager
import yaml
import serial
import time
import threading
import serial.tools.list_ports


step = 0.005
min_cap = -0.1
max_cap = 1.0

# Load configuration
with open('pump_config.yaml', 'r') as file:
    config = yaml.safe_load(file)
    initial_value = config['CHANNEL_CONFIGS']['pump'].get('idle', min_cap)  # Default to min_cap if not specified

# Initialize the pump driver
pump_driver = masi_driver.ExcavatorController(deadzone=0, simulation_mode=False, inputs=1, input_rate_threshold=1,
                                              config_file='pump_config.yaml')

# Initialize the RPM measurement
rpm_sensor = sensor_manager.RPMSensor(sensor_name='sensor1', config_file='sensor_config.yaml')


# Function to find the serial device
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
    print("No serial device found")
    ser = None


# Stop the ESC beep if first start
def reset_pump():
    value = -1.0
    delay = 1.0
    pump_driver.update_values(value, min_cap=min_cap)
    print(f"Sent reset command to pump ({value})")
    time.sleep(delay)
    print(f"Waited {delay} seconds after reset")


reset_pump()

# Initialize pygame
pygame.init()

# Set up display
width, height = 800, 600
window = pygame.display.set_mode((width, height))
pygame.display.set_caption('Pump and RPM Monitor')

# Set up fonts
font = pygame.font.Font(None, 36)

# Initial values
value = initial_value
last_value = value
clock = pygame.time.Clock()
pump_stopped = False

# User input variables
input_box = pygame.Rect(50, 200, 140, 36)
#color_inactive = pygame.Color('lightskyblue3')


input_boxes = {
    "P": pygame.Rect(50, 250, 140, 36),
    "I": pygame.Rect(50, 330, 140, 36),
    "D": pygame.Rect(50, 410, 140, 36),
    "setpoint": pygame.Rect(50, 490, 140, 36),
    "initial_angle1": pygame.Rect(50, 570, 140, 36),
    "max_angle1": pygame.Rect(50, 650, 140, 36),
    "initial_angle2": pygame.Rect(250, 570, 140, 36),
    "max_angle2": pygame.Rect(250, 650, 140, 36)
}
user_text = {key: '' for key in input_boxes.keys()}
active_box = None

# Limits for PID values, setpoints, and angles
limits = {
    'P': (0.0, 10.0),
    'I': (0.0, 10.0),
    'D': (0.0, 10.0),
    'setpoint': (-100.0, 100.0),
    'initial_angle1': (0, 180),
    'max_angle1': (0, 180),
    'initial_angle2': (0, 180),
    'max_angle2': (0, 180)
}

# Storage for parsed serial data
serial_data = {
    'Input1': 0.0,
    'Setpoint1': 0.0,
    'Output1': 0.0,
    'Kp1': 0.0,
    'Ki1': 0.0,
    'Kd1': 0.0,
    'Input2': 0.0,
    'Setpoint2': 0.0,
    'Output2': 0.0,
    'Kp2': 0.0,
    'Ki2': 0.0,
    'Kd2': 0.0,
    'initial_angle1': 90,
    'max_angle1': 120,
    'initial_angle2': 90,
    'max_angle2': 120
}

# Flag to stop the serial thread
stop_serial_thread = False


# Function to read serial data
def read_serial_data():
    """Read messages from the device continuously."""
    while ser and ser.is_open and not stop_serial_thread:
        if ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            if message:
                print("Received:", message)
                try:
                    if message.startswith("Input1 (KalY):"):
                        data = message.split('\t')
                        serial_data['Input1'] = float(data[0].split(': ')[1])
                        serial_data['Setpoint1'] = float(data[1].split(': ')[1])
                        serial_data['Output1'] = float(data[2].split(': ')[1])
                        serial_data['Kp1'] = float(data[3].split(': ')[1])
                        serial_data['Ki1'] = float(data[4].split(': ')[1])
                        serial_data['Kd1'] = float(data[5].split(': ')[1])
                    elif message.startswith("Input2 (KalY2):"):
                        data = message.split('\t')
                        serial_data['Input2'] = float(data[0].split(': ')[1])
                        serial_data['Setpoint2'] = float(data[1].split(': ')[1])
                        serial_data['Output2'] = float(data[2].split(': ')[1])
                        serial_data['Kp2'] = float(data[3].split(': ')[1])
                        serial_data['Ki2'] = float(data[4].split(': ')[1])
                        serial_data['Kd2'] = float(data[5].split(': ')[1])
                    elif message.startswith("initial_angle1="):
                        serial_data['initial_angle1'] = int(message.split('=')[1])
                    elif message.startswith("max_angle1="):
                        serial_data['max_angle1'] = int(message.split('=')[1])
                    elif message.startswith("initial_angle2="):
                        serial_data['initial_angle2'] = int(message.split('=')[1])
                    elif message.startswith("max_angle2="):
                        serial_data['max_angle2'] = int(message.split('=')[1])
                except Exception as e:
                    print(f"Error parsing serial data: {e}")


# Start thread for reading serial data
if ser:
    thread_read = threading.Thread(target=read_serial_data)
    thread_read.daemon = True
    thread_read.start()

# Main loop
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP and not pump_stopped:
                value += step
            elif event.key == pygame.K_DOWN and not pump_stopped:
                value -= step
            elif event.key == pygame.K_SPACE:
                if pump_stopped:
                    # Start the pump with the last set value
                    value = last_value
                    pump_stopped = False
                else:
                    # Stop the pump
                    last_value = value
                    value = min_cap
                    pump_stopped = True
            elif event.key == pygame.K_RETURN and active_box:
                try:
                    new_value = float(user_text[active_box])
                    if limits[active_box][0] <= new_value <= limits[active_box][1]:
                        command = f"{active_box}={new_value}\n"
                        if ser:
                            ser.write(command.encode())
                            print(f"Sent {active_box}: {new_value}")
                    else:
                        print(f"{active_box} value out of limits.")
                except ValueError:
                    print("Invalid input. Please enter a float.")
                user_text[active_box] = ''
                active_box = None
            elif active_box:
                if event.key == pygame.K_BACKSPACE:
                    user_text[active_box] = user_text[active_box][:-1]
                else:
                    user_text[active_box] += event.unicode
        elif event.type == pygame.MOUSEBUTTONDOWN:
            active_box = None
            for box_name, rect in input_boxes.items():
                if rect.collidepoint(event.pos):
                    active_box = box_name

    # Clamp value between min_cap and max_cap
    if not pump_stopped:
        value = max(min(value, max_cap), min_cap)

    # Update pump driver
    pump_driver.update_values(value, min_cap=min_cap)

    # Clear screen
    window.fill((0, 0, 0))

    # Render text
    rpm_text = font.render(f'RPM: {rpm_sensor.get_rpm():.2f}', True, (255, 255, 255))
    value_text = font.render(f'Pump Value: {value:.3f}', True, (255, 255, 255))
    instructions_text = font.render("Use UP/DOWN to change value, SPACE to stop/start", True, (255, 255, 255))

    window.blit(rpm_text, (50, 50))
    window.blit(value_text, (50, 100))
    window.blit(instructions_text, (50, 150))

    # Display serial data
    input1_text = font.render(f"Input1: {serial_data['Input1']:.2f}", True, (255, 255, 255))
    setpoint1_text = font.render(f"Setpoint1: {serial_data['Setpoint1']:.2f}", True, (255, 255, 255))
    output1_text = font.render(f"Output1: {serial_data['Output1']:.2f}", True, (255, 255, 255))
    kp1_text = font.render(f"Kp1: {serial_data['Kp1']:.2f}", True, (255, 255, 255))
    ki1_text = font.render(f"Ki1: {serial_data['Ki1']:.2f}", True, (255, 255, 255))
    kd1_text = font.render(f"Kd1: {serial_data['Kd1']:.2f}", True, (255, 255, 255))

    input2_text = font.render(f"Input2: {serial_data['Input2']:.2f}", True, (255, 255, 255))
    setpoint2_text = font.render(f"Setpoint2: {serial_data['Setpoint2']:.2f}", True, (255, 255, 255))
    output2_text = font.render(f"Output2: {serial_data['Output2']:.2f}", True, (255, 255, 255))
    kp2_text = font.render(f"Kp2: {serial_data['Kp2']:.2f}", True, (255, 255, 255))
    ki2_text = font.render(f"Ki2: {serial_data['Ki2']:.2f}", True, (255, 255, 255))
    kd2_text = font.render(f"Kd2: {serial_data['Kd2']:.2f}", True, (255, 255, 255))

    window.blit(input1_text, (50, 300))
    window.blit(setpoint1_text, (50, 340))
    window.blit(output1_text, (50, 380))
    window.blit(kp1_text, (50, 420))
    window.blit(ki1_text, (50, 460))
    window.blit(kd1_text, (50, 500))

    window.blit(input2_text, (250, 300))
    window.blit(setpoint2_text, (250, 340))
    window.blit(output2_text, (250, 380))
    window.blit(kp2_text, (250, 420))
    window.blit(ki2_text, (250, 460))
    window.blit(kd2_text, (250, 500))

    # Draw input boxes
    for box_name, rect in input_boxes.items():
        pygame.draw.rect(window, (255, 0, 0) if active_box == box_name else (255, 255, 255), rect, 2)
        text_surface = font.render(user_text[box_name], True, (255, 255, 255))
        window.blit(text_surface, (rect.x + 5, rect.y + 5))

    # Display initial and max angles
    initial_angle1_text = font.render(f"Initial Angle1: {serial_data['initial_angle1']}", True, (255, 255, 255))
    max_angle1_text = font.render(f"Max Angle1: {serial_data['max_angle1']}", True, (255, 255, 255))
    initial_angle2_text = font.render(f"Initial Angle2: {serial_data['initial_angle2']}", True, (255, 255, 255))
    max_angle2_text = font.render(f"Max Angle2: {serial_data['max_angle2']}", True, (255, 255, 255))

    window.blit(initial_angle1_text, (450, 300))
    window.blit(max_angle1_text, (450, 340))
    window.blit(initial_angle2_text, (450, 380))
    window.blit(max_angle2_text, (450, 420))

    # Update display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

# Stop the serial thread
stop_serial_thread = True

if ser:
    ser.close()

pygame.quit()
