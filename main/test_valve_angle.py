# This file is used to find the center position of the valves, measure the dead area,
# and display various sensor readings in real-time.

import pygame
from PWM_controller import PWM_hat
import yaml
from time import sleep
from ADC_sensors import ADC_hat

# Configuration file path
CONFIG_FILE = 'test_bench_config.yaml'

# Control parameters
step = 0.005  # Step size for pump control
min_cap = -1.0  # Minimum cap for pump and servo values
max_cap = 1.0  # Maximum cap for pump and servo values
servo_step = 0.01  # Step size for servo input

# Lists to store values and readings
values = []  # Pump and servo values
pressure_readings = []  # Pressure sensor readings

# Load configuration from YAML file
with open(CONFIG_FILE, 'r') as file:
    config = yaml.safe_load(file)
    initial_value = config['CHANNEL_CONFIGS']['pump'].get('idle', min_cap)  # Default to min_cap if not specified
    servo_angle_limit = config['CHANNEL_CONFIGS']['test_servo'].get('multiplier_positive',
                                                                    30)  # Default to ±30 degrees if not specified

# Initialize the pump driver
pwm_driver = PWM_hat(simulation_mode=False,
                     deadzone=0,
                     inputs=2,
                     input_rate_threshold=0,
                     config_file='test_bench_config.yaml')

# Initialize the pressure sensors
adc_sensors = ADC_hat(simulation_mode=False,
                      config_file='sensor_config.yaml')
                      
                      
adc_sensors.reset_angle_range()


# Function to stop the ESC beep if first start
def reset_pwm():
    values = [-1.0, 0.0]
    pwm_driver.update_values(values, min_cap=min_cap)
    print(f"Sent reset command to pump ({values})")


reset_pwm()

# Show user the used input mappings
pwm_driver.print_input_mappings()

sleep(5)

# Initialize pygame
pygame.init()

# Set up display
width, height = 800, 700  # Increased height to accommodate new information
window = pygame.display.set_mode((width, height))
pygame.display.set_caption('Pump and Servo Control')

# Set up fonts
font = pygame.font.Font(None, 36)
small_font = pygame.font.Font(None, 24)

# Initial values
pump_value = initial_value
last_pump_value = pump_value
servo_input = 0.0  # Starting servo input (centered)
clock = pygame.time.Clock()
pump_stopped = False

# Main loop
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RIGHT and not pump_stopped:
                pump_value = min(pump_value + step, max_cap)
            elif event.key == pygame.K_LEFT and not pump_stopped:
                pump_value = max(pump_value - step, min_cap)
            elif event.key == pygame.K_UP:
                servo_input = min(servo_input + servo_step, max_cap)
            elif event.key == pygame.K_DOWN:
                servo_input = max(servo_input - servo_step, min_cap)
            elif event.key == pygame.K_SPACE:
                if pump_stopped:
                    pump_value = last_pump_value
                    pump_stopped = False
                else:
                    last_pump_value = pump_value
                    pump_value = min_cap
                    pump_stopped = True
            elif event.key == pygame.K_u:
                # Update config
                pwm_driver.update_config(CONFIG_FILE)
                # Reload initial values from the updated config
                with open(CONFIG_FILE, 'r') as file:
                    config = yaml.safe_load(file)
                    initial_value = config['CHANNEL_CONFIGS']['pump'].get('idle', min_cap)
                    servo_angle_limit = config['CHANNEL_CONFIGS']['test_servo'].get('multiplier_positive', 30)
                pump_value = initial_value
                servo_input = 0.0  # Reset servo input to center
                print("Configuration updated")

    # Update pump driver and servo input
    values = [pump_value, servo_input]

    # Get servo angles from the driver
    servo_angles = pwm_driver.update_values(values, min_cap=min_cap, return_servo_angles=True)
    
    #servo_angle = angles["test_servo"]

    # Read sensors (scaled and filtered)
    pressure_readings = adc_sensors.read_scaled(read="pressure")
    filtered_pressure_readings = adc_sensors.read_filtered(read="pressure")
    angle_readings = adc_sensors.read_scaled(read="angle")
    filtered_angle_readings = adc_sensors.read_filtered(read="angle")

    # Get angle ranges
    angle_ranges = adc_sensors.get_angle_range()

    # Clear the window
    window.fill((0, 0, 0))

    # Render main control values
    pump_text = font.render(f'Pump Value: {pump_value:.3f}', True, (255, 255, 255))
    servo_text = font.render(f'Servo Input: {servo_input:.3f}', True, (255, 255, 255))
    valve_text = font.render(f'Valve Angle: {servo_angles["test_servo angle"]:.1f}°', True, (255, 255, 255))
    boomAngle_text = font.render(f'Boom Angle: {angle_readings["LiftBoom angle"]:.1f}°', True, (255, 255, 255))

    window.blit(pump_text, (50, 50))
    window.blit(servo_text, (50, 100))
    window.blit(valve_text, (50, 150))
    window.blit(boomAngle_text, (275, 150))

    # Render instructions
    instructions = [
        "L/R arrows: Adjust pump",
        "U/D arrows: Adjust servo",
        "SPACE: Stop/Resume pump",
        "U key: Update configuration"
    ]
    for i, instruction in enumerate(instructions):
        text = small_font.render(instruction, True, (255, 255, 255))
        window.blit(text, (50, 200 + i * 25))

    # Render pressure readings
    pressure_box = pygame.Rect(50, 325, 400, 200)
    pygame.draw.rect(window, (100, 100, 100), pressure_box)
    pressure_title = font.render("Pressure Readings", True, (255, 255, 255))
    window.blit(pressure_title, (60, 335))

    y_offset = 375
    for sensor_name, pressure in pressure_readings.items():
        pressure_text = small_font.render(f"{sensor_name}: {pressure:.2f} (scaled)", True, (255, 255, 255))
        window.blit(pressure_text, (60, y_offset))
        y_offset += 25

        filtered_pressure = filtered_pressure_readings.get(sensor_name)
        if filtered_pressure is not None:
            filtered_text = small_font.render(f"  Filtered: {filtered_pressure:.2f}", True, (200, 200, 200))
            window.blit(filtered_text, (60, y_offset))
        y_offset += 30

    # Render angle readings
    angle_box = pygame.Rect(460, 325, 300, 200)
    pygame.draw.rect(window, (100, 100, 100), angle_box)
    angle_title = font.render("Angle Readings", True, (255, 255, 255))
    window.blit(angle_title, (470, 335))

    y_offset = 375
    for sensor_name, angle in angle_readings.items():
        if 'angle' in sensor_name.lower():
            angle_text = small_font.render(f"{sensor_name}: {angle:.2f}° (scaled)", True, (255, 255, 255))
            window.blit(angle_text, (470, y_offset))
            y_offset += 25

            filtered_angle = filtered_angle_readings.get(sensor_name)
            if filtered_angle is not None:
                filtered_text = small_font.render(f"  Filtered: {filtered_angle:.2f}°", True, (200, 200, 200))
                window.blit(filtered_text, (470, y_offset))
            y_offset += 30

    # Render angle ranges
    range_box = pygame.Rect(50, 535, 710, 50)
    pygame.draw.rect(window, (100, 100, 100), range_box)
    range_title = font.render("Angle Ranges", True, (255, 255, 255))
    window.blit(range_title, (60, 545))

    range_text = small_font.render(f"{angle_ranges}", True, (255, 255, 255))
    window.blit(range_text, (60, 575))

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(15)  # 15 FPS max for 16-bit ADC

# Quit pygame when the loop ends
pygame.quit()
