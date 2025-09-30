import numpy as np

class TrajPlanner:
    """
    Trajectory Planner class for calculating trajectories for different polynomial orders and relevant coefficients.
    """

    def __init__(self, setpoints):
        """
        Initialize the TrajPlanner class.

        Parameters:
        setpoints (numpy array): List of setpoints to travel to.
        """
        self.setpoints = setpoints

    ## Implement the required methods below. ##
    """
    calc_cubic_coeff()
    Input: start time, end time, start position, end position, start velocity, and end velocity
    Output: 1 x 4 numpy array specifying coefficient for the desired cubic trajectory
    Description: calculates the coefficient for the desired cubic trajectory
    """
    def calc_cubic_coeff(self, t0, tf, p0, pf, v0, vf):
            A = np.array([[1, t0, t0**2, t0**3],
                        [0, 1, 2 * t0, 3 * t0**2],
                        [1, tf, tf**2, tf**3],
                        [0, 1, 2 * tf, 3 * tf**2]])
            
            b = np.array([p0, v0, pf, vf])  
            a = np.linalg.solve(A, b)
            print('Coefficients:', a)
            return a

    """
    calc_cubic_traj()
    Input: trajectory time between two setpoints, number of intermediate waypoints (n), and
    a set of cubic coefficients
    Output: (n + 2) x 1 numpy array specifying planned waypoints for a cubic trajectory
    Description: calculates the waypoints for one setpoint dimension for a cubic trajectory
    """
    def calc_cubic_traj(self, traj_time, n, coeff):
        # Generate time steps for waypoints
        t = np.linspace(0, traj_time, n + 2)
        
        # Calculate waypoints for cubic trajectory
        return coeff[0] + coeff[1] * t + coeff[2] * t**2 + coeff[3] * t**3

    """
    get_cubic_traj()
    Input: trajectory time between two setpoints, and number of intermediate waypoints (n)
    Output: (n+2)x5 numpy array specifying planned waypoints for all four joints, including
    time
    Description: calculates the waypoints for all four setpoint dimensions for cubic trajectories. The method internally calls the calc_cubic_coeff() and calc_cubic_traj
    for each of the dimensions. The initial and final velocities should be set to zero.
    """
    def get_cubic_traj(self, traj_time, n):

        total_waypoints = np.zeros((n + 2, 5))
        times = np.linspace(0, traj_time, n + 2)
        total_waypoints[:, 0] = times
        
        # Iterate through each joint
        for i in range(4):
            p0 = self.setpoints[0, i]
            pf = self.setpoints[1, i]
            coeffs = self.calc_cubic_coeff(0, traj_time, p0, pf, 0, 0)
            waypoints = self.calc_cubic_traj(traj_time, n, coeffs)
            total_waypoints[:, i + 1] = waypoints
        
        return total_waypoints

# # Usage Example
# import numpy as np
# from traj_planner import TrajPlanner

# # Define setpoints (example values)
# setpoints = np.array([
#     [15, -45, -60, 90],
#     [-90, 15, 30, -45]
# ])

# # Create a TrajPlanner object
# trajectories = TrajPlanner(setpoints)

# # Generate cubic trajectory
# cubic_traj = trajectories.get_cubic_traj(traj_time=5, points_num=10)
# print(cubic_traj)
