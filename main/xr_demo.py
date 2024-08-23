import requests
import time
import random

import ADC_sensors
import PWM_controller

NUM_SEND = 8
# pump pressure
# boom extraction
# boom retraction
# pump control value
# servo angle
# boom angle
# button
# slider

#SERVER_URL = 'http://192.168.0.131:8000'
SERVER_URL = 'http://127.0.0.1:8000'

def send_values(data):
    try:
        response = requests.post(f'{SERVER_URL}/send_data/client2', json=data)
        if response.status_code == 200:
            return data
        else:
            print(f"Failed to send values. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")


def get_latest_values():
    try:
        response = requests.get(f'{SERVER_URL}/receive_data/client2')
        if response.status_code == 200:
            data = response.json()
            return data
        else:
            print(f"Failed to get values. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")




if __name__ == "__main__":
    while True:
        values = [random.random() for _ in range(NUM_SEND)]
        data = {'values': values}
        send_values(data)
        print(f"sent: {data.get('values')}")

        data = get_latest_values()
        print(f"received: {data.get('values')}")

        time.sleep(0.05)

        # init pwm and adc
        # update the send / recv if there is some
        # run the control
