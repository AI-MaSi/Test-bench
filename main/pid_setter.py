from PID import PID
import ADC_sensors
import PWM_controller

import time
import random

SIMULATION = False
LOOP_HZ = 10  # Control loop frequency in Hz
TOLERANCE = 1 # +- degrees
TIME_GOAL = 10 # seconds needed to stay until new setpoint is generated

PUMP_IDLE = 0.060


def read_sensors(adc):
    angles = adc.read_filtered(read="angle")
    pressures = adc.read_filtered(read="pressure")
    return angles["LiftBoom angle"], pressures["Pump"]

def calibrate_boom(adc,pwm):
    print("Starting calibration...")

    # Drive boom up
    pwm.update_values([0.070, 1])
    time.sleep(1)  # Wait for pressure to stabilize

    while True:
        current_angle, current_pressure = read_sensors(adc)
        print(f"[UP] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
        if current_pressure >= 200:
            pwm.update_values([0.070, 0])  # Stop
            break
        time.sleep(0.1)

    # Drive boom down
    pwm.update_values([0.070, -1])
    time.sleep(1)  # Wait for pressure to stabilize

    while True:
        current_angle, current_pressure = read_sensors(adc)
        print(f"[DOWN] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
        if current_pressure >= 200:
            pwm.update_values([0.070, 0])  # Stop
            break
        time.sleep(0.1)

    # Get calibrated range
    ranges = adc.get_angle_range()
    min_angle, max_angle, min_raw, max_raw = ranges["LiftBoom angle"]
    print(f"Calibrated angle range: {min_angle:.2f} ({min_raw:.2f}) to {max_angle:.2f} ({max_raw:.2f})")

    # Move to midpoint
    mid_angle = (min_angle + max_angle) / 2
    print(f"Moving to mid point: {mid_angle:.2f}")

    while True:
        current_angle, _ = read_sensors(adc)
        if abs(current_angle - mid_angle) < 1:  # 1 degree tolerance
            pwm.update_values([0.070, 0])  # Stop
            print(f"Arm positioned at middle angle: {current_angle:.1f}")
            break
        elif current_angle < mid_angle:
            pwm.update_values([0.070, -0.5])  # Move up slowly
        else:
            pwm.update_values([0.070, 0.5])  # Move down slowly
        time.sleep(0.1)

    print("Calibration complete!")

def read_pid_values(filename="pid_config.txt"):
    """Read PID values from a configuration file."""
    pid_values = {}
    with open(filename, 'r') as file:
        for line in file:
            if "=" in line:
                key, value = line.strip().split('=')
                pid_values[key] = float(value)
    return pid_values


def main(LOOP_HZ, TOLERANCE, TIME_GOAL, setpoint, pid_controller, pump_pid_controller, adc, pwm):
    time_at_setpoint = 0  # Initialize time spent at setpoint
    last_pid_values = read_pid_values()

    while True:
        # Get the current angle and pressure from the sensor
        current_angle, current_pressure = read_sensors(adc)

        print(f"Current angle: {current_angle:.1f} / target: {setpoint:.1f}")

        # Read and update PID values from file if they have changed
        current_pid_values = read_pid_values()
        if current_pid_values != last_pid_values:
            pid_controller.setKp(current_pid_values['P'])
            pid_controller.setKi(current_pid_values['I'])
            pid_controller.setKd(current_pid_values['D'])
            pump_pid_controller.setKp(current_pid_values['Pump_P'])
            pump_pid_controller.setKi(current_pid_values['Pump_I'])
            pump_pid_controller.setKd(current_pid_values['Pump_D'])
            last_pid_values = current_pid_values
            print(f"Updated PID values: P={current_pid_values['P']}, I={current_pid_values['I']}, D={current_pid_values['D']}")
            print(f"Updated Pump PID values: P={current_pid_values['Pump_P']}, I={current_pid_values['Pump_I']}, D={current_pid_values['Pump_D']}")

        # Update the main PID controller
        pid_controller.SetPoint = setpoint
        pid_controller.update(current_angle)
        action = -pid_controller.output
        print(f"[PID] action: {action:.2f}")

        # Calculate pump speed adjustment based on the distance from setpoint
        pump_pid_controller.SetPoint = 0  # Target is zero error
        pump_pid_controller.update(abs(setpoint - current_angle))
        pump_speed = PUMP_IDLE + abs(pump_pid_controller.output)
        print(f"[Pump PID] speed: {pump_speed:.2f}")


        # Update PWM with the action and pump speed
        pwm.update_values([pump_speed, action])

        # Check if the current angle is within the tolerance range
        if abs(current_angle - setpoint) <= TOLERANCE:
            time_at_setpoint += 1 / LOOP_HZ  # Increment time at setpoint
        else:
            time_at_setpoint = 0  # Reset time if out of tolerance

        # If the setpoint is held within tolerance for the specified time, return
        if time_at_setpoint >= TIME_GOAL:
            print(f"Setpoint {setpoint} reached and held for {TIME_GOAL} seconds.")
            return  # Exit the function

        # Wait for the next loop iteration
        time.sleep(1 / LOOP_HZ)


if __name__ == "__main__":
    # get PID values from .txt file
    pid_values = read_pid_values()

    # init PID for position control
    pid_controller = PID(P=pid_values['P'], I=pid_values['I'], D=pid_values['D'])
    pid_controller.setSampleTime(1.0 / LOOP_HZ)
    pid_controller.setMaxOutput(1.0)
    pid_controller.setWindup(1.0)

    # init PID for pump speed control
    pump_pid_controller = PID(P=pid_values['Pump_P'], I=pid_values['Pump_I'], D=pid_values['Pump_D'])
    pump_pid_controller.setSampleTime(1.0 / LOOP_HZ)
    pump_pid_controller.setMaxOutput(0.05)
    pump_pid_controller.setWindup(1.0)

    # init PWM controller
    pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                                 inputs=2,
                                 simulation_mode=SIMULATION,
                                 pump_variable=True,  # Enable pump control
                                 tracks_disabled=True,
                                 input_rate_threshold=0,
                                 deadzone=0)

    # init sensors
    adc = ADC_sensors.ADC_hat(config_file='sensor_config.yaml',
                              decimals=4,
                              simulation_mode=SIMULATION,
                              min_sim_voltage=0.5,
                              max_sim_voltage=4.5,
                              frequency=0.1)

    adc.list_sensors()
    time.sleep(2)

    calibrate_boom(adc, pwm)

    while True:
        try:
            # Generate a new random setpoint between 5-70 degrees
            setpoint = round(random.uniform(5, 70), 2)
            print(f"New setpoint: {setpoint:.1f}!")
            time.sleep(2)

            # Run the control loop for the current setpoint
            main(LOOP_HZ, TOLERANCE, TIME_GOAL, setpoint, pid_controller, pump_pid_controller, adc, pwm)
        except KeyboardInterrupt:
            pwm.update_values([-1.0, 0.0])
            print("Goodbye!")
            break