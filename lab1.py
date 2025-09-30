import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time
import numpy
import matplotlib.pyplot  as plt

#Robot Setup
traj_time = 10
robot = Robot()
robot.write_time(traj_time)
robot.write_motor_state(True)

robot.write_joints([0, 0, 0, 0])
time.sleep(traj_time)

# Initilize Joint position and Time Arrays
t = []
joint = []

# Robot Movement
base_waypoints = [45]  # Define base waypoints

for base_waypoint in base_waypoints:  # Iterate through waypoints
    robot.write_joints([base_waypoint, 0, 0, 0])  # Write joint values
    start_time = time.time()  # Start timer
    while time.time() - start_time < traj_time:
        joint.append(robot.get_joints_readings()[0])        
        t.append(time.time())

motor11 = []
motor12 = []
motor13 = []
motor14 = []
for pos in joint:
    motor11.append(pos[0])
for pos in joint:
    motor12.append(pos[1])
for pos in joint:
    motor13.append(pos[2])
for pos in joint:
    motor14.append(pos[3])
    
t_array = numpy.array(t)
joint_array = numpy.array(joint)
motor11_array = numpy.array(motor11)
motor12_array = numpy.array(motor12)
motor13_array = numpy.array(motor13)
motor14_array = numpy.array(motor14)

#data = numpy.empty(t_array, joint)


    # Create a line plot of temperature over time
plt.figure(figsize=(10, 6))

plt.subplot(4, 1, 1)
plt.plot(t_array, motor11_array)
plt.title('Motor 11 Position')
plt.xlabel("Time (s)")
plt.ylabel("Position (Deg)")

plt.subplot(4, 1, 2)
plt.plot(t_array, motor12_array)
plt.title('Motor 12 Position')
plt.xlabel("Time (s)")
plt.ylabel("Position (Deg)")

plt.subplot(4, 1, 3)
plt.plot(t_array, motor13_array)
plt.title('Motor 13 Position')
plt.xlabel("Time (s)")
plt.ylabel("Position (Deg)")

plt.subplot(4, 1, 4)
plt.plot(t_array, motor14_array)
plt.title('Motor 14 Position')
plt.xlabel("Time (s)")
plt.ylabel("Position (Deg)")
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.hist(numpy.diff(t_array))
plt.title('Motor 1 Time Interval Frequency')
plt.xlabel("Time Interval (s)")
plt.ylabel("Frequency")
plt.grid(True)
plt.show()
#print(t_array)
#print(motor1_array)

time.sleep(1)