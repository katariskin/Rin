# Rinimport numpy as np
import matplotlib.pyplot as plt

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

    def update(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# Simulation parameters
dt = 0.01  # time step
sim_time = 10  # seconds
steps = int(sim_time / dt)

# Airplane parameters (simplified)
pitch_angle = 0  # initial pitch angle (degrees)
pitch_rate = 0   # initial pitch rate (degrees/sec)
inertia = 1  # moment of inertia (arbitrary units)
damping = 0.2  # damping factor

# Desired setpoint (target pitch angle)
target_pitch = 10  # degrees

# PID controller parameters
pid = PID(kp=2.0, ki=0.5, kd=1.0, setpoint=target_pitch)

# Data storage for plotting
time_vec = []
pitch_vec = []
control_vec = []

for step in range(steps):
    time = step * dt
    # PID control
    control_input = pid.update(pitch_angle, dt)
    
    # Airplane pitch dynamics (simplified)
    # θ'' = (1/inertia) * (control_input - damping * θ')
    pitch_acc = (control_input - damping * pitch_rate) / inertia
    pitch_rate += pitch_acc * dt
    pitch_angle += pitch_rate * dt

    # Store data
    time_vec.append(time)
    pitch_vec.append(pitch_angle)
    control_vec.append(control_input)

# Plotting results
plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.plot(time_vec, pitch_vec)
plt.title('Pitch Angle')
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (deg)')

plt.subplot(1,2,2)
plt.plot(time_vec, control_vec)
plt.title('Control Input')
plt.xlabel('Time (s)')
plt.ylabel('Elevator Command')

plt.tight_layout()
plt.show(),
