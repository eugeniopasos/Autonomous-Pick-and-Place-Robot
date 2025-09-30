import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time
import numpy as np
import matplotlib.pyplot  as plt
import pickle

# Robot Setup
traj_time = 2.5
robot = Robot()
robot.write_time(traj_time)
robot.write_motor_state(True)

robot.write_joints([0, 0, 0, 0])
time.sleep(traj_time)

# Initilize Joint position and Time Arrays
t = []
joint = []
ee = []

# Robot Movement
# Define base waypoints
waypoints = [[ -75.96375653,  -39.73508666,    6.17652105,   93.55856561],
             [0, 10, 50, -45],
             [0, 10,  0, -80],
             [0, -45, 60, 50]]  

for waypoint in waypoints:          # Iterate through waypoints
    robot.write_joints(waypoint)    # Write joint values
    start_time = time.time()        # Start timer
    while time.time() - start_time < traj_time:
        # Make lists of joint positions, time, and end effector positions
        joint.append(robot.get_joints_readings()[0])        
        t.append(time.time())
        a = np.radians(robot.get_joints_readings()[0])
        ee.append(robot.get_ee_pos(a))



data_to_save = {
    'joint_angles': np.array([joint]),
    'ee_positions': np.array([ee]),
    'timestamps': np.array([t])
}

# Save Data to pickle File
def save_to_pickle(data, filename):
    with open(filename, 'wb') as file:
        pickle.dump(data, file)

# Loading data from a pickle file
def load_from_pickle(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)

save_to_pickle(data_to_save, 'robot_data_1.pkl')

loaded_data = load_from_pickle('robot_data_1.pkl')
joint = loaded_data['joint_angles']
ee = loaded_data['ee_positions']
t = loaded_data['timestamps']

# Sort data and plot joint positions
def motor_plots(joint,t):
    motor11 = []
    motor12 = []
    motor13 = []
    motor14 = []
    print(joint)
    for pos in joint:
        motor11.append(pos[0])
    for pos in joint:
        motor12.append(pos[1])
    for pos in joint:
        motor13.append(pos[2])
    for pos in joint:
        motor14.append(pos[3])

    t_array = np.array(t)
    motor11_array = np.array(motor11)
    motor12_array = np.array(motor12)
    motor13_array = np.array(motor13)
    motor14_array = np.array(motor14)

    plt.figure(figsize=(10, 6))

    plt.subplot(4, 1, 1)
    plt.plot(t_array[0], motor11_array)
    plt.title('Motor 11 Position')
    plt.xlabel("Time (s)")
    plt.ylabel("Position (Deg)")
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(t_array[0], motor12_array)
    plt.title('Motor 12 Position')
    plt.xlabel("Time (s)")
    plt.ylabel("Position (Deg)")
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(t_array[0], motor13_array)
    plt.title('Motor 13 Position')
    plt.xlabel("Time (s)")
    plt.ylabel("Position (Deg)")
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(t_array[0], motor14_array)
    plt.title('Motor 14 Position')
    plt.xlabel("Time (s)")
    plt.ylabel("Position (Deg)")
    plt.grid(True)
    plt.show()

# Sort data and plot X and Z positons vs time
x = []
y = []
z = []
for e in ee[0]:
    x.append(e[0])
for e in ee[0]:
    y.append(e[1])
for e in ee[0]:
    z.append(e[2])

x_array = np.array(x)
y_array = np.array(y)
z_array = np.array(z)
t_array = np.array(t-t[0][0])
j_array = np.array(joint[0])
print(j_array)
motor_plots(j_array, t_array)

plt.figure(figsize=(10, 6))

plt.plot(t_array[0], x_array, color ='g', label = 'X Position')
plt.plot(t_array[0], z_array, color ='r', label = 'Z Position')
plt.title('X and Z Position vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (mm)')
plt.grid(True)
plt.legend()
plt.show()

# Plot 2D Trajectory Plot of X-Z Plane
plt.figure(figsize=(10, 6))
plt.plot(x_array, z_array, color ='g')
plt.title('2D Trajectory Plot of X-Z Plane')
plt.xlabel('X Position (mm)')
plt.ylabel('Z Position (mm)')
plt.grid(True)
plt.show()

# Plot 2D Trajectory Plot of X-Y Plane
plt.figure(figsize=(10, 6))
plt.plot(x_array, y_array, color ='g')
plt.title('2D Trajectory Plot of X-Y Plane')
plt.xlabel('X Position (mm)')
plt.ylabel('Y Position (mm)')
plt.grid(True)
plt.show()

time.sleep(1)