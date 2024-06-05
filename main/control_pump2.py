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
pump_driver = masi_driver.ExcavatorController(deadzone=0, simulation_mode=False, inputs=1, input_rate_threshold=1, config_file='pump_config.yaml')

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
pump_stopped = True  # Set the pump to be stopped initially

# User input variables
input_box = pygame.Rect(50, 200, 140, 36)
user_text = ''
active = False

# Function to read serial data
def read_serial_data():
    """Read messages from the device continuously."""
    while ser and ser.is_open:
        if ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            if message:
                print("Received:", message)

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
            elif event.key == pygame.K_RETURN and active:
                try:
                    setpoint = int(user_text)
                    command = f"{setpoint}\n"
                    if ser:
                        ser.write(command.encode())
                        print(f"Sent setpoint: {setpoint}")
                except ValueError:
                    print("Invalid input. Please enter an integer.")
                user_text = ''
            elif active:
                if event.key == pygame.K_BACKSPACE:
                    user_text = user_text[:-1]
                else:
                    user_text += event.unicode
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if input_box.collidepoint(event.pos):
                active = True
            else:
                active = False

    # Clamp value between min_cap and max_cap
    if not pump_stopped:
        value = max(min_cap, min(value, max_cap))

    # Get RPM
    rpm = rpm_sensor.read_rpm()

    # Update pump value
    pump_driver.update_values(value, min_cap=min_cap)

    # Clear the screen
    window.fill((0, 0, 0))

    # Render text
    rpm_text = font.render(f'RPM: {rpm}', True, (255, 255, 255))
    value_text = font.render(f'Pump Value: {value:.3f}', True, (255, 255, 255))
    user_text_surface = font.render(user_text, True, (255, 255, 255))

    # Blit text to the screen
    window.blit(rpm_text, (50, 50))
    window.blit(value_text, (50, 100))
    pygame.draw.rect(window, (255, 255, 255), input_box, 2)
    window.blit(user_text_surface, (input_box.x + 5, input_box.y + 5))

    if pump_stopped:
        stop_text = font.render('STOP', True, (255, 0, 0))
        window.blit(stop_text, (50, 150))

    # Update the display
    pygame.display.flip()

    # Maintain 10Hz loop rate
    clock.tick(10)

# Stop monitoring thread
pump_driver.stop_monitoring()
# Reset pump
pump_driver.reset(pump_reset_point=-0.1)
# Close serial connection
if ser:
    ser.close()
# Quit pygame
pygame.quit()
