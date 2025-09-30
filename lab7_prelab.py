import matplotlib.pyplot as plt
import numpy as np
import PIDController
# Initialize controller
controller = PIDController.PIDController(dim=3, dt=0.05)

# Time vector (5 seconds of simulation)
t = np.linspace(0, 5, 101)

# Initialize arrays to store results
errors = np.zeros((len(t), 3))
outputs = np.zeros((len(t), 3))
initial_error = np.array([10., 8., 6.])

# Simulate system response
for i in range(len(t)):
    # Store current error
    errors[i] = initial_error

    # Compute control output
    outputs[i] = controller.compute_pid(initial_error)

    # Simulate system response (error reduction)
    initial_error = initial_error * 0.95  # Simple decay

plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(t, errors[:, 0], label='Error X')
plt.plot(t, errors[:, 1], label='Error Y')
plt.plot(t, errors[:, 2], label='Error Z')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (mm)')
plt.title('Error vs Time')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, outputs[:, 0], label='Output X')
plt.plot(t, outputs[:, 1], label='Output Y')
plt.plot(t, outputs[:, 2], label='Output Z')
plt.xlabel('Time (s)')
plt.ylabel('Control Output')
plt.title('Control Output vs Time')
plt.legend()

plt.tight_layout()
plt.show()