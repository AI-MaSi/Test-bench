import time
from pynput.keyboard import Controller

# Initialize the keyboard controller
keyboard = Controller()

def press_space(wait_time):
    # Press the spacebar
    keyboard.press(' ')
    keyboard.release(' ')
    print("Pressed spacebar")

    # Wait for the specified time
    time.sleep(wait_time)

    # Press the spacebar again
    keyboard.press(' ')
    keyboard.release(' ')
    print("Pressed spacebar again")

if __name__ == "__main__":
    # Set the wait time in seconds
    wait_time = 5  # Change this value to the desired wait time

    press_space(wait_time)
