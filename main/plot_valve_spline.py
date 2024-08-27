import ADC_sensors
import PWM_controller

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Function to calculate angular velocity
def calculate_angular_velocity(current_angle, last_angle=None, last_angle_time=None):
    current_time = time.time()

    if last_angle is None or last_angle_time is None:
        last_angle = current_angle
        last_angle_time = current_time
        return 0, last_angle, last_angle_time  # We don't have enough information for the first calculation

    time_diff = current_time - last_angle_time

    if time_diff == 0:
        return 0, last_angle, last_angle_time  # Avoid division by zero

    angle_diff = current_angle - last_angle

    # Calculate velocity in deg/s
    velocity = angle_diff / time_diff

    # Update last values for next calculation
    last_angle = current_angle
    last_angle_time = current_time

    return velocity, last_angle, last_angle_time


# Initialize the sensor and controller
sensors = ADC_sensors.ADC_hat(config_file='sensor_config.yaml',
                              decimals=4,
                              simulation_mode=True,
                              min_sim_voltage=0.5,
                              max_sim_voltage=4.5,
                              frequency=0.1)

controller = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                                    inputs=2,
                                    simulation_mode=True,
                                    pump_variable=False,
                                    tracks_disabled=True,
                                    input_rate_threshold=0,
                                    deadzone=0)

# Initialize data arrays
valve_values = []
pressures = []
rotation_speeds = []

# Initialize variables for angular velocity calculation
last_angle = None
last_angle_time = None

# Set up the plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
line1, = ax1.plot([], [], 'r-', label='Pressure')
line2, = ax2.plot([], [], 'b-', label='Rotation Speed')

ax1.set_xlabel('Valve Control Value')
ax1.set_ylabel('Pressure')
ax1.set_title('Pressure vs Valve Control Value')
ax1.legend()

ax2.set_xlabel('Valve Control Value')
ax2.set_ylabel('Rotation Speed')
ax2.set_title('Rotation Speed vs Valve Control Value')
ax2.legend()


# Function to read sensor data and control servo
def read_sensors_and_control(valve_value, last_angle, last_angle_time):
    # Pressure
    pressures = sensors.read_filtered(read='pressure')
    pump_pressure = pressures['Pump']

    # Angle
    angles = sensors.read_filtered(read='angle')

    boom_angle = angles['LiftBoom angle']

    # Rotation
    rotation_speed, last_angle, last_angle_time = calculate_angular_velocity(boom_angle, last_angle, last_angle_time)

    # Valve control
    valve_angles = controller.update_values((0.070, valve_value), return_servo_angles=True)

    # Test servo angle
    servo_angle = valve_angles["test_servo angle"]


    return pump_pressure, rotation_speed, last_angle, last_angle_time


# Animation update function
def update(frame):
    global last_angle, last_angle_time

    valve_value = frame * 0.1  # Increase valve value over time

    # Loop to drive the boom until pressure exceeds 150 psi
    moving = False
    pressure_up = 0
    speed_up = 0

    while pressure_up <= 150:
        pressure_up, speed_up, last_angle, last_angle_time = read_sensors_and_control(valve_value, last_angle,
                                                                                      last_angle_time)

        # Check if the boom is moving
        if abs(speed_up) < 0.01:  # Threshold to check if movement is significant
            print("Boom not moving significantly. Increasing valve value.")
            valve_value += 0.1  # Increase the valve value if the boom is not moving
            if valve_value > 1:
                print("Maximum valve value reached, but boom is not moving.")
                break
        else:
            moving = True
            break

    if not moving:
        print("Exiting loop due to no movement.")
        return line1, line2

    valve_values.append(valve_value)
    pressures.append(pressure_up)
    rotation_speeds.append(speed_up)

    # Move boom down (opposite direction)
    pressure_down, speed_down, last_angle, last_angle_time = read_sensors_and_control(-valve_value, last_angle,
                                                                                      last_angle_time)
    valve_values.append(-valve_value)
    pressures.append(pressure_down)
    rotation_speeds.append(speed_down)

    # Update plots
    line1.set_data(valve_values, pressures)
    line2.set_data(valve_values, rotation_speeds)

    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    return line1, line2


# Create animation
anim = FuncAnimation(fig, update, frames=100, interval=100, blit=True)

# Show the plot
plt.tight_layout()
plt.show()

# Save the final plot as an image
plt.savefig('hydraulic_valve_flow_rate_spline.png')
