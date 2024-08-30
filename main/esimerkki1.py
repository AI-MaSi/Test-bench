import ADC_sensors

import PWM_controller


pwm = PWM_controller.PWM_hat(config_file='test_bench_config.yaml')

adc = ADC_sensors.ADC_hat(config_file='sensor_config.yaml')


