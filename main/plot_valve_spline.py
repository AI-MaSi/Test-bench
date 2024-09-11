import numpy as np
import matplotlib.pyplot as plt
import time

import ADC_sensors
import PWM_controller

IS_SIMULATION = True
TEST_DURATION = 120  # Duration of test in seconds

# Initialize the PWM controller and ADC sensors
pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                             inputs=2,  # pump, valve1
                             simulation_mode=IS_SIMULATION,
                             pump_variable=True,
                             tracks_disabled=True,
                             input_rate_threshold=0,
                             deadzone=0)
adc = ADC_sensors.ADC_hat(config_file='sensor_config.yaml',
                          decimals=4,
                          simulation_mode=IS_SIMULATION,
                          min_sim_voltage=0.5,
                          max_sim_voltage=4.5,
                          frequency=0.1)


def calibrate():
    TOLERANCE = 1  # 1 degree tolerance for calibration
    print("Starting calibration...")
    adc.reset_angle_range()

    # Drive boom up
    pwm.update_values([0.070, 1])
    time.sleep(1)  # Wait for pressure to stabilize

    while True:
        current_angle, current_pressure = read_sensors()
        print(f"[UP] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
        if current_pressure >= 200:
            pwm.update_values([0.070, 0])  # Stop
            break
        time.sleep(0.1)

    # Drive boom down
    pwm.update_values([0.070, -1])
    time.sleep(1)  # Wait for pressure to stabilize

    while True:
        current_angle, current_pressure = read_sensors()
        print(f"[DOWN] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
        if current_pressure >= 200:
            pwm.update_values([0.070, 0])  # Stop
            break
        time.sleep(0.1)

    # Get calibrated range
    ranges = adc.get_angle_range()
    min_angle, max_angle, min_raw, max_raw = ranges["LiftBoom angle"]
    print(f"Calibrated angle range: {min_angle:.2f} to {max_angle:.2f}")

    # Move to midpoint
    mid_angle = (min_angle + max_angle) / 2
    print(f"Moving to mid point: {mid_angle:.2f}")

    while True:
        current_angle, _ = read_sensors()
        if abs(current_angle - mid_angle) < TOLERANCE:  # 1 degree tolerance
            pwm.update_values([0.070, 0])  # Stop
            print(f"Arm positioned at middle angle: {current_angle:.1f}")
            break
        elif current_angle < mid_angle:
            pwm.update_values([0.070, -0.5])  # Move up slowly
        else:
            pwm.update_values([0.070, 0.5])  # Move down slowly
        time.sleep(0.1)

    print("Calibration complete!")


# Placeholder for game controller input
def get_controller_input():
    # Replace this with your game controller input code
    # This should return a value between -1 and 1
    return 0  # Placeholder return value


def read_sensors():
    angles = adc.read_filtered(read="angle")
    pressures = adc.read_filtered(read="pressure")
    return angles["LiftBoom angle"], pressures["Pump"]


def collect_data(duration=TEST_DURATION):
    valve_values = []
    pressures = []

    start_time = time.time()
    while time.time() - start_time < duration:
        valve_value = get_controller_input()
        pwm.update_values([0.070, valve_value])  # Keep the pump speed constant and change only the valve value
        _, pressure = read_sensors()    # We don't care about the angle here

        valve_values.append(valve_value)
        pressures.append(pressure)

        time.sleep(0.1)  # Collect data every 0.1 seconds

    return valve_values, pressures


def normalize_data(valve_values, pressures):
    valve_percent = np.array(valve_values) * 100  # Valve control value is -1...1, so easy conversion

    # Get the pressure range
    pressure_range = adc.get_pressure_range()
    min_pressure, max_pressure = pressure_range["Pump"]

    # Normalize pressure to 0-100% based on the observed range
    if max_pressure > min_pressure:
        pressure_percent = (np.array(pressures) - min_pressure) / (max_pressure - min_pressure) * 100
    else:
        pressure_percent = np.zeros_like(pressures)

    return valve_percent, pressure_percent


def plot_results(valve_percent, pressure_percent):
    plt.figure(figsize=(10, 6))

    # Calculate median pressure for each unique valve value
    unique_valve_values = np.unique(valve_percent)
    median_pressures = [np.median(pressure_percent[valve_percent == v]) for v in unique_valve_values]

    plt.plot(unique_valve_values, median_pressures, 'b-', linewidth=2)

    plt.xlabel('Valve Input (%)')
    plt.ylabel('Relative Pressure (%)')
    plt.title('Valve Input vs Relative Pressure')
    plt.grid(True)
    plt.xlim(-100, 100)
    plt.ylim(0, 100)

    plt.tight_layout()
    plt.savefig('valve_pressure_plot.png', dpi=300)
    plt.close()


def main():
    print(f"Collecting data for {TEST_DURATION} seconds...")
    valve_values, pressures = collect_data()

    print("Processing and plotting data...")
    valve_percent, pressure_percent = normalize_data(valve_values, pressures)
    plot_results(valve_percent, pressure_percent)

    print("Plot saved as valve_pressure_plot.png")


if __name__ == "__main__":
    main()