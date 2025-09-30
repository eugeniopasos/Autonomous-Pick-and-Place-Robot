from TrajPlanner import TrajPlanner
import numpy as np

# Define the setpoints
setpoints = np.array([[15, -45, -60, 90], [-90, 15, 30, -45]])

# Initialize the TrajPlanner
planner = TrajPlanner(setpoints)

# Generate cubic trajectory
traj_time = 5  # seconds
n_waypoints = 6
trajectory = planner.get_cubic_traj(traj_time, n_waypoints)

# Print the trajectory
print("Generated Trajectory:")
print(trajectory)