import requests
import time
import tkinter as tk
from tkinter import messagebox
import threading
import queue

NUM_VALUES = 8
SERVER_URL = 'http://192.168.0.131:8000'
# localhost
# SERVER_URL = 'http://localhost:8000'


def send_values(data):
    try:
        response = requests.post(f'{SERVER_URL}/send_data/client1', json=data, timeout=5)
        if response.status_code == 200:
            return data
        else:
            print(f"Failed to send values. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")


def get_latest_values():
    try:
        response = requests.get(f'{SERVER_URL}/receive_data/client1', timeout=5)
        if response.status_code == 200:
            data = response.json()
            return data
        else:
            print(f"Failed to get values. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")


class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.master.title("Hydraulic System Control")
        self.pack(padx=10, pady=10)
        self.queue = queue.Queue()
        self.create_widgets()
        self.update_thread = None
        self.stop_event = threading.Event()
        self.thread_lock = threading.Lock()  # Add a lock to prevent race conditions

    def create_widgets(self):
        self.fields = [
            "Pump Pressure", "Boom Extension", "Boom Retraction", "Valve control value",
            "Valve Angle", "Boom Angle", "Button", "Setpoint"
        ]
        self.vars = {}
        self.entries = {}

        for i, field in enumerate(self.fields):
            tk.Label(self, text=f"{field}:").grid(row=i, column=0, sticky='e')
            var = tk.StringVar()
            self.vars[field] = var

            if field in ["Button", "Setpoint"]:
                entry = tk.Entry(self, textvariable=var)
                entry.grid(row=i, column=1, sticky='w')
                self.entries[field] = entry
            else:
                tk.Label(self, textvariable=var).grid(row=i, column=1, sticky='w')

        self.send_button = tk.Button(self, text="Send Values", command=self.send_values)
        self.send_button.grid(row=NUM_VALUES, column=0, columnspan=2, pady=10)

        self.status_var = tk.StringVar()
        self.status_label = tk.Label(self, textvariable=self.status_var)
        self.status_label.grid(row=NUM_VALUES + 1, column=0, columnspan=2)

    def send_values(self):
        try:
            button = int(self.vars["Button"].get())
            setpoint = float(self.vars["Setpoint"].get())

            if button not in [0, 1]:
                raise ValueError("Button must be 0 or 1")
            if not 0 <= setpoint <= 90:
                raise ValueError("Setpoint must be between 0 and 90")

            values = [None] * NUM_VALUES
            values[6] = button
            values[7] = setpoint

            data = {'values': values}

            def send_task():
                result = send_values(data)
                if result:
                    self.queue.put(("status", "Values sent successfully"))
                else:
                    self.queue.put(("status", "Failed to send values"))

            threading.Thread(target=send_task, daemon=True).start()
            self.status_var.set("Sending values...")

        except ValueError as e:
            messagebox.showerror("Input Error", str(e))

    def get_latest_values(self):
        while not self.stop_event.is_set():
            data = get_latest_values()
            if data and 'values' in data:
                received = data.get('values')
                if received is None:
                    self.queue.put(("status", "No data available"))
                elif len(received) == NUM_VALUES:
                    self.queue.put(("update", received))
                else:
                    self.queue.put(("status", f"Received data does not contain {NUM_VALUES} values"))
            else:
                self.queue.put(("status", "Failed to receive data"))
            time.sleep(1)

    def start_periodic_update(self):
        with self.thread_lock:  # Prevent starting the thread multiple times
            if self.update_thread is None or not self.update_thread.is_alive():
                self.update_thread = threading.Thread(target=self.get_latest_values, daemon=True)
                self.update_thread.start()
        self.process_queue()

    def process_queue(self):
        try:
            while True:
                message = self.queue.get_nowait()
                if message[0] == "update":
                    received = message[1]
                    for i, field in enumerate(self.fields):
                        if field not in ["Button", "Setpoint"]:  # Don't update user input fields
                            value = received[i]
                            if value is None:
                                self.vars[field].set("N/A")
                            else:
                                self.vars[field].set(f"{value:.2f}" if isinstance(value, float) else str(value))
                    self.status_var.set("Data updated")
                elif message[0] == "status":
                    self.status_var.set(message[1])
        except queue.Empty:
            pass
        finally:
            self.master.after(100, self.process_queue)

    def cleanup(self):
        self.stop_event.set()
        if self.update_thread:
            self.update_thread.join(timeout=1)
        self.queue.queue.clear()  # Clear the queue to prevent stale messages

    def on_closing(self):
        self.cleanup()
        self.master.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = Application(master=root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.start_periodic_update()
    root.mainloop()
