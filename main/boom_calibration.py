"""
This script is used to calibrate the test bench boom arm.
The calibration process involves moving the boom arm up and down to determine the minimum and maximum angles that the boom arm can reach.
The calibration process also involves moving the boom arm to the midpoint between the minimum and maximum angles.

Usage:

    import boom_calibration
    min_angle, max_angle = boom_calibration.calibrate(adc, pwm)

    The calibrate function will return the minimum and maximum angles of the boom arm.
"""

from time import sleep

PUMP_SPEED = 0.090

def read_sensors(adc):
    angles = adc.read_filtered(read="angle")
    pressures = adc.read_filtered(read="pressure")
    return angles["LiftBoom angle"], pressures["Pump"]

def calibrate(adc, pwm, tolerance=1):
    print("Starting calibration...")
    adc.reset_angle_range()

    # Drive boom up
    pwm.update_values([PUMP_SPEED, 1])
    sleep(1)  # Wait for pressure to stabilize

    while True:
        current_angle, current_pressure = read_sensors(adc)
        print(f"[UP] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
        if current_pressure >= 200:
            pwm.update_values([PUMP_SPEED, 0])  # Stop
            break
        sleep(0.1)

    # Drive boom down
    pwm.update_values([PUMP_SPEED, -1])
    sleep(1)  # Wait for pressure to stabilize

    while True:
        current_angle, current_pressure = read_sensors(adc)
        print(f"[DOWN] Pressure: {current_pressure:.1f}, Current angle: {current_angle:.2f}")
        if current_pressure >= 200:
            pwm.update_values([PUMP_SPEED, 0])  # Stop
            break
        sleep(0.1)

    # Get calibrated range
    ranges = adc.get_angle_range()
    min_angle, max_angle, min_raw, max_raw = ranges["LiftBoom angle"]
    print(f"Calibrated angle range: {min_angle:.2f} to {max_angle:.2f}")

    # Move to midpoint
    mid_angle = (min_angle + max_angle) / 2
    print(f"Moving to mid point: {mid_angle:.2f}")

    while True:
        current_angle, _ = read_sensors(adc)
        if abs(current_angle - mid_angle) < tolerance:
            pwm.update_values([PUMP_SPEED, 0])  # Stop
            print(f"Arm positioned at middle angle: {current_angle:.1f}")
            break
        elif current_angle < mid_angle:
            pwm.update_values([PUMP_SPEED, -0.5])  # Move up slowly
        else:
            pwm.update_values([PUMP_SPEED, 0.5])  # Move down slowly
        sleep(0.1)

    print("Calibration complete!")
    return min_angle, max_angle