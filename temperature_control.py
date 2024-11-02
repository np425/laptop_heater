import time
import psutil
import threading
import numpy as np


# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0.0
        self.integral = 0.0
        self.integral_limit = 1000  # Limit for integral term to prevent windup

    def calculate(self, current_value):
        error = self.setpoint - current_value

        # Proportional term
        proportional = self.kp * error

        # Integral term with windup protection
        self.integral += error
        self.integral = max(
            min(self.integral, self.integral_limit), -self.integral_limit
        )
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.previous_error)
        self.previous_error = error

        # PID output
        output = proportional + integral + derivative
        return output


# Temperature Control Class with Multi-Threading and Enhanced Accuracy
class TemperatureController:
    def __init__(self, target_temperature=60.0, kp=3.0, ki=0.15, kd=0.01):
        self.target_temperature = target_temperature
        self.core_pids = []
        self.threads = []
        self.running = True
        self._initialize_controllers(kp, ki, kd)

    def _initialize_controllers(self, kp, ki, kd):
        core_temperatures = self.get_per_core_temperatures()
        if core_temperatures:
            # Initialize a PID controller for each core and start a thread for each
            self.core_pids = [
                PIDController(kp=kp, ki=ki, kd=kd, setpoint=self.target_temperature)
                for _ in core_temperatures
            ]
            for i in range(len(core_temperatures)):
                thread = threading.Thread(target=self._control_loop, args=(i,))
                self.threads.append(thread)
                thread.start()

    def get_per_core_temperatures(self):
        try:
            temps = psutil.sensors_temperatures()
            core_temps = []
            if "coretemp" in temps:
                core_temps = [entry.current for entry in temps["coretemp"]]
            elif "cpu-thermal" in temps:
                core_temps = [entry.current for entry in temps["cpu-thermal"]]
            elif "acpitz" in temps:
                core_temps = [entry.current for entry in temps["acpitz"]]
            return core_temps if core_temps else None
        except Exception as e:
            print("Unable to retrieve temperature data:", e)
        return None

    def get_core_temperatures(self):
        """
        Retrieves the current temperature of each CPU core.
        """
        core_temperatures = self.get_per_core_temperatures()
        if core_temperatures:
            return [
                {"core": i, "temperature": temp}
                for i, temp in enumerate(core_temperatures)
            ]
        return []

    def set_target_temperature(self, new_target):
        # Update target temperature for all PID controllers
        self.target_temperature = float(new_target)
        for pid in self.core_pids:
            pid.setpoint = self.target_temperature

    def set_pid_parameters(self, kp, ki, kd):
        """
        Update PID parameters for all cores, allowing PSO tuning to set these values dynamically.
        """
        for pid in self.core_pids:
            pid.kp = kp
            pid.ki = ki
            pid.kd = kd

    def _control_loop(self, core_index):
        print(f"Starting control loop for Core {core_index}.")
        previous_temperatures = []

        while self.running:
            core_temperatures = self.get_per_core_temperatures()
            if core_temperatures is None:
                print("Temperature sensor not available.")
                return

            # Moving average to smooth out temperature readings
            temp = core_temperatures[core_index]
            previous_temperatures.append(temp)
            if len(previous_temperatures) > 5:
                previous_temperatures.pop(0)
            smoothed_temp = sum(previous_temperatures) / len(previous_temperatures)

            pid_controller = self.core_pids[core_index]
            load_adjustment = pid_controller.calculate(smoothed_temp)
            load_percentage = min(
                max(load_adjustment, 0), 100
            )  # Constrain between 0-100%

            # Calculate and print the temperature difference from the target
            temperature_difference = smoothed_temp - self.target_temperature
            difference_sign = "+" if temperature_difference > 0 else ""
            print(
                f"Core {core_index} Temp: {smoothed_temp:.2f}°C ({difference_sign}{temperature_difference:.2f}°C from target), "
                f"Adjusted Load: {load_percentage:.2f}%"
            )

            self._adaptive_cpu_load(load_percentage)

    def _adaptive_cpu_load(self, load_percentage):
        start_time = time.perf_counter()  # Use high-resolution timer
        load_duration = load_percentage / 100  # Portion of time to stay busy
        
        size = 1000  # Matrix size for high computational load
        a = np.random.rand(size, size)
        b = np.random.rand(size, size)

        # CPU-intensive task: Continuous multiplication to simulate load
        while (time.perf_counter() - start_time) < load_duration:
            np.dot(a, b)

        # Calculate the actual time spent in the busy loop using perf_counter
        busy_time = time.perf_counter() - start_time
        remaining_time = max(0, 1 - busy_time)

        # If there's any remaining time, sleep for that duration
        if remaining_time > 0:
            time.sleep(remaining_time)

    def simulate_response(self):
        """
        Simulates the temperature response to the current PID parameters and returns cumulative error.
        This method is intended to be used by the PSO tuner for evaluating the fitness of PID parameters.
        """
        temperature = 30.0  # Initial temperature
        cumulative_error = 0.0
        time_steps = 100  # Number of time steps for the simulation

        for _ in range(time_steps):
            pid_output = self.core_pids[0].calculate(
                temperature
            )  # Use first core PID for simulation
            temperature += pid_output * 0.1  # Simulated response adjustment
            cumulative_error += abs(self.target_temperature - temperature)

        return cumulative_error

    def stop(self):
        print("Stopping all control loops.")
        self.running = False
        for thread in self.threads:
            thread.join()
        print("All control loops stopped.")


# Example usage
if __name__ == "__main__":
    controller = TemperatureController(target_temperature=60.0)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.stop()
