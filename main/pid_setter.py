import time
import random
import matplotlib.pyplot as plt
from PID import PID
import ADC_sensors
import PWM_controller
import boom_calibration

SIMULATION = False
TOLERANCE = 2  # +- degrees
TIME_GOAL = 7  # seconds needed to stay until new setpoint is generated
PUMP_IDLE = 0.050


def read_sensors(adc):
    angles = adc.read_filtered(read="angle")
    pressures = adc.read_filtered(read="pressure")
    return angles["LiftBoom angle"], pressures["Pump"]


def read_pid_values(filename="pid_config.txt"):
    """Read PID values from a configuration file."""
    pid_values = {}
    try:
        with open(filename, 'r') as file:
            for line in file:
                if "=" in line:
                    key, value = line.strip().split('=')
                    pid_values[key] = float(value)
        return pid_values
    except Exception as e:
        print(f"Error reading PID values: {e}")
        raise


def run_pid_control(setpoint, pid_controller, pump_pid_controller, adc, pwm):
    time_at_setpoint = 0
    start_time = time.time()
    time_data, setpoint_data, current_angle_data, pump_speed_data = [], [], [], []

    while True:
        current_time = time.time() - start_time
        current_angle, _ = read_sensors(adc)

        time_data.append(current_time)
        setpoint_data.append(setpoint)
        current_angle_data.append(current_angle)

        # Update PID controllers
        pid_controller.SetPoint = setpoint
        pid_controller.update(current_angle)
        action = -pid_controller.output  # Use output directly

        pump_pid_controller.SetPoint = 0
        pump_pid_controller.update(abs(setpoint - current_angle))
        pump_output = pump_pid_controller.output

        # Calculate pump speed
        pump_speed = PUMP_IDLE + abs(pump_output)
        pump_speed_data.append(pump_speed)

        # Update PWM
        pwm.update_values([pump_speed, action])

        # Check if setpoint is reached
        if abs(current_angle - setpoint) <= TOLERANCE:
            time_at_setpoint += 0.1  # Assuming 10Hz loop
        else:
            time_at_setpoint = 0

        if time_at_setpoint >= TIME_GOAL:
            print(f"Setpoint {setpoint} reached and held for {TIME_GOAL} seconds.")
            break

        time.sleep(0.1)  # 10Hz loop

    return time_data, setpoint_data, current_angle_data, pump_speed_data


def save_step_response_graph(time_data, setpoint_data, current_angle_data, pump_speed_data, run_number, pid_values):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # Plot angle data
    ax1.plot(time_data, setpoint_data, label='Setpoint')
    ax1.plot(time_data, current_angle_data, label='Current Angle')
    ax1.set_ylabel('Angle (degrees)')
    ax1.legend()
    ax1.set_title(f'Step Response - Run {run_number}')

    # Plot pump speed data
    ax2.plot(time_data, pump_speed_data, label='Pump Speed')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Pump Speed')
    ax2.legend()

    # Add PID values and loop frequency to the plot
    pid_info = f"Position PID: P={pid_values['P']:.4f}, I={pid_values['I']:.4f}, D={pid_values['D']:.4f}\n" \
               f"Pump PID: P={pid_values['Pump_P']:.4f}, I={pid_values['Pump_I']:.4f}, D={pid_values['Pump_D']:.4f}\n" \
               f"Loop Frequency: {pid_values['LOOP_HZ']:.2f} Hz"
    fig.text(0.1, 0.01, pid_info, fontsize=10, verticalalignment='bottom')

    plt.tight_layout()
    plt.savefig(f'step_response_graph_{run_number}.png')
    plt.close()


def main():
    # Initialize ADC and PWM
    #TODO: test with more pump speed and less valve multiplier!
    pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml',
                                 inputs=2,
                                 simulation_mode=SIMULATION,
                                 pump_variable=True,
                                 tracks_disabled=True,
                                 input_rate_threshold=0,
                                 deadzone=0)
    adc = ADC_sensors.ADC_hat(config_file='sensor_config.yaml',
                              decimals=4,
                              simulation_mode=SIMULATION,
                              min_sim_voltage=0.5,
                              max_sim_voltage=4.5,
                              frequency=0.1)

    # Get initial PID values
    pid_values = read_pid_values()

    # Initialize PID controllers
    pid_controller = PID(P=pid_values['P'], I=pid_values['I'], D=pid_values['D'])
    pid_controller.setMaxOutput(1.0)
    pid_controller.setWindup(1.0)

    pump_pid_controller = PID(P=pid_values['Pump_P'], I=pid_values['Pump_I'], D=pid_values['Pump_D'])
    pump_pid_controller.setMaxOutput(0.12)
    pump_pid_controller.setWindup(0.2)
    pump_pid_controller.set_rampup(True, 0.06)

    # Set sample time for both controllers
    pid_controller.setSampleTime(1.0 / pid_values['LOOP_HZ'])
    pump_pid_controller.setSampleTime(1.0 / pid_values['LOOP_HZ'])

    # Calibrate boom
    min_angle, max_angle = boom_calibration.calibrate(adc, pwm, tolerance=TOLERANCE)

    run_number = 1
    while True:
        # Generate random setpoint
        end_slack = 3
        setpoint = round(random.uniform(min_angle + end_slack, max_angle - end_slack), 2)
        print(f"New setpoint: {setpoint:.1f}")

        # Run PID control and record values
        time_data, setpoint_data, current_angle_data, pump_speed_data = run_pid_control(
            setpoint, pid_controller, pump_pid_controller, adc, pwm
        )

        # Save step response graph
        save_step_response_graph(time_data, setpoint_data, current_angle_data, pump_speed_data, run_number, pid_values)

        run_number += 1

        # Optional: add a delay between runs or a way to exit the loop
        user_input = input("Press Enter to continue or 'q' to quit: ")
        if user_input.lower() == 'q':
            break

    # Cleanup
    pwm.update_values([-1.0, 0.0])  # Reset pump and servo
    print("Process complete. Exiting.")


if __name__ == "__main__":
    main()