# this file is used to find the center position of the valves, and measure the dead area

import pygame
from PWM_controller import PWM_hat
import yaml
from time import sleep
from ADC_sensors import ADC_hat

CONFIG_FILE = 'test_bench_config.yaml'

step = 0.005
min_cap = -1.0
max_cap = 1.0
servo_step = 0.01  # Step for servo input

values = [] # Pump and servo values
pressure_readings = []


# Load configuration
with open(CONFIG_FILE, 'r') as file:
    config = yaml.safe_load(file)
    initial_value = config['CHANNEL_CONFIGS']['pump'].get('idle', min_cap)  # Default to min_cap if not specified
    servo_angle_limit = config['CHANNEL_CONFIGS']['test_servo'].get('multiplier_positive', 30)   # Default to +-30 degrees if not specified

# Initialize the pump driver
pwm_driver = PWM_hat(deadzone=0, simulation_mode=False, inputs=2, input_rate_threshold=1,
                                             config_file='test_bench_config.yaml')

# Initialize the pressure sensors
adc_sensors = ADC_hat(config_file='sensor_config.yaml')

# Function to stop the ESC beep if first start
def reset_pwm():
    values = [-1.0, 0.0]
    pwm_driver.update_values(values, min_cap=min_cap)
    print(f"Sent reset command to pump ({values})")

reset_pwm()

# show user the used input mappings
pwm_driver.print_input_mappings()

sleep(5)

# Initialize pygame
pygame.init()

# Set up display
width, height = 800, 600
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

    # wow if you now ask nicely the driver will return the servo angles:)
    servo_angles = pwm_driver.update_values(values, min_cap=min_cap, return_servo_angles=True)

    # Read pressure sensors
    pressure_readings = adc_sensors.read_pressure(alpha=1.0, return_names=True)
    # output will be: sensor_idx_name, sensor_config_name, value
    angle_readings = adc_sensors.read_angle(alpha=1.0)

    window.fill((0, 0, 0))

    pump_text = font.render(f'Pump Value: {pump_value:.3f}', True, (255, 255, 255))
    servo_text = font.render(f'Servo Input: {servo_input:.3f}', True, (255, 255, 255))
    valve_text = font.render(f'Valve Angle: {servo_angles[0]:.1f}°', True, (255, 255, 255))
    boomAngle_text = font.render(f'Boom Angle: {angle_readings[0][1]:.1f}°', True, (255, 255, 255))

    instructions_text1 = small_font.render("L/R arrows: Adjust pump", True, (255, 255, 255))
    instructions_text2 = small_font.render("U/D arrows: Adjust servo", True, (255, 255, 255))
    instructions_text3 = small_font.render("SPACE: Stop/Resume pump", True, (255, 255, 255))
    instructions_text4 = small_font.render("U key: Update configuration", True, (255, 255, 255))

    # Render values
    window.blit(pump_text, (50, 50))
    window.blit(servo_text, (50, 100))
    window.blit(valve_text, (50, 150))
    window.blit(boomAngle_text, (275, 150))

    # Render instructions
    window.blit(instructions_text1, (50, 200))
    window.blit(instructions_text2, (50, 225))
    window.blit(instructions_text3, (50, 250))
    window.blit(instructions_text4, (50, 275))

    # Render pressure readings
    pressure_box = pygame.Rect(50, 325, 300, 150)
    pygame.draw.rect(window, (100, 100, 100), pressure_box)
    pressure_title = font.render("Pressure Readings", True, (255, 255, 255))
    window.blit(pressure_title, (60, 335))

    y_offset = 375
    for sensor_id, sensor_name, pressure in pressure_readings:
        # includes psi to bar conversion
        if pressure is not None:
            pressure_text = small_font.render(f"{sensor_name}: {(pressure * 0.06894757):.2f} bar", True,
                                              (255, 255, 255))
        else:
            pressure_text = small_font.render(f"{sensor_name}: N/A", True, (255, 255, 255))
        window.blit(pressure_text, (60, y_offset))
        y_offset += 30

    pygame.display.flip()

    # Cap the frame rate
    clock.tick(15) # 15 SPS max for 16-bit ADC

pygame.quit()