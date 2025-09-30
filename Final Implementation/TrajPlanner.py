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

    def calc_cubic_coeff(self, t0, tf, p0, pf, v0, vf):
        """
        Given the initial time, final time, initial position, final position, initial velocity, and final velocity,
        returns cubic polynomial coefficients.

        Parameters:
        t0 (float): Start time of trajectory
        tf (float): End time of trajectory
        p0 (float): Initial setpoint
        pf (float): Final setpoint
        v0 (float): Initial velocity
        vf (float): Final velocity

        Returns:
        numpy array: The calculated polynomial coefficients.
        """
        coeff_matrix = np.array([
            [1, t0, t0 ** 2, t0 ** 3],
            [0, 1, 2 * t0, 3 * t0 ** 2],
            [1, tf, tf ** 2, tf ** 3],
            [0, 1, 2 * tf, 3 * tf ** 2]
        ])
        qs = np.array([p0, v0, pf, vf])
        coeff = np.linalg.solve(coeff_matrix, qs)

        return coeff


    def calc_cubic_traj(self, traj_time, points_num, coeff):
        """
        Given the time between setpoints, number of points between waypoints, and polynomial coefficients,
        returns the cubic trajectory of waypoints for a single pair of setpoints.

        Parameters:
        traj_time (int): Time between setPoints.
        points_num (int): Number of waypoints between setpoints.
        coeff (numpy array): Polynomial coefficients for trajectory.

        Returns:
        numpy array: The calculated waypoints.
        """
        waypoints = np.zeros(points_num)
        times = np.linspace(0, traj_time, points_num+2)[1:-1]

        for k, t in enumerate(times):
            waypoints[k] = coeff[0] + coeff[1] * t + coeff[2] * t**2 + coeff[3] * t**3
        
        return waypoints


    def get_cubic_traj(self, traj_time, points_num):
        """
        Given the time between setpoints and number of points between waypoints, returns the cubic trajectory.

        Parameters:
        traj_time (int): Time between setPoints.
        points_num (int): Number of waypoints between setpoints.

        Returns:
        numpy array: List of waypoints for the cubic trajectory.
        """
        setpoints = self.setpoints
        waypoints_list = np.zeros(((len(setpoints)-1)*(points_num+1)+1, 5))
        
        for i in range(4):
            count = 0
            for j in range(len(setpoints)-1):
                coeff = self.calc_cubic_coeff(0, traj_time, setpoints[j, i], setpoints[j+1, i], 0, 0)
                waypoints_list[count, 1:] = setpoints[j, :]
                count += 1
                waypoints_list[count:count+points_num, i+1] = self.calc_cubic_traj(traj_time, points_num, coeff)
                count += points_num
            waypoints_list[count, 1:] = setpoints[-1, :]

        time = np.linspace(0, traj_time*(len(setpoints)-1), waypoints_list.shape[0])
        waypoints_list[:, 0] = time
        return waypoints_list

    def calc_quintic_coeff(self, t0, tf, p0, pf, v0, vf, a0, af):
        """
        calc_quintic_coeff()
        Input: start time, end time, start position, end position, start velocity, end velocity, start
        acceleration, and end acceleration
        Output: 1 x 6 numpy array specifying coefficient for the desired quintic trajectory
        Description: calculates the coefficient for the desired quintic trajectory
        """
        # Set up the system of equations as Ax = b
        A = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [1, tf, tf**2, tf**3, tf**4, tf**5],
            [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
            [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
        
        b = np.array([p0, v0, a0, pf, vf, af])
        
        # Solve for the coefficients
        coeff = np.linalg.solve(A, b)
        
        return coeff

    def calc_quintic_traj(self, traj_time, points_num, coeff):
        """
        calc_quintic_traj()
        Input: trajectory time between two setpoints, number of intermediate waypoints (n), and
        a set of quintic coefficients
        Output: (n + 2) x 1 numpy array specifying planned waypoints for a quintic trajectory
        Description: calculates the waypoints for one setpoint dimension for a quintic trajectory
        """
        waypoints = np.zeros(points_num)
        times = np.linspace(0, traj_time, points_num+2)[1:-1]

        for k, t in enumerate(times):
            waypoints[k] = coeff[0] + coeff[1] * t + coeff[2] * t**2 + coeff[3] * t**3 + coeff[4] * t**4 + coeff[5] * t**5
        
        return waypoints
    
    def get_quintic_traj(self, traj_time, points_num):
    
        """
        get_quintic_traj()
        Input: trajectory time between two setpoints, and number of intermediate waypoints (n)
        Output: (n+2)x5 numpy array specifying planned waypoints for all four joints, including
        time as the first column.
        Description: calculates the waypoints for all four setpoint dimensions for quintic trajectories. The method internally calls the calc_quintic_coeff() and calc_quintic_traj
        for each of the dimensions. The initial and final velocities and accelerations should be
        set to zero.
        Note: The method should handle multi-setpoint trajectories seamlessly.
        """
        setpoints = self.setpoints
        waypoints_list = np.zeros(((len(setpoints)-1)*(points_num+1)+1, 5))
        
        for i in range(4):
            count = 0
            for j in range(len(setpoints)-1):
                coeff = self.calc_quintic_coeff(0, traj_time, setpoints[j, i], setpoints[j+1, i], 0, 0, 0, 0)
                waypoints_list[count, 1:] = setpoints[j, :]
                count += 1
                waypoints_list[count:count+points_num, i+1] = self.calc_quintic_traj(traj_time, points_num, coeff)
                count += points_num
            waypoints_list[count, 1:] = setpoints[-1, :]

        time = np.linspace(0, traj_time*(len(setpoints)-1), waypoints_list.shape[0])
        waypoints_list[:, 0] = time
        return waypoints_list

# # Usage Example
# import numpy as np
# from traj_planner import TrajPlanner

# # Define setpoints (example values)
# setpoints = np.array([
#     [0, 0, 0, 0],
#     [10, 20, 30, 40],
#     [20, 30, 40, 50]
# ])

# # Create a TrajPlanner object
# traj_planner = TrajPlanner(setpoints)

# # Generate cubic trajectory
# cubic_traj = traj_planner.get_cubic_traj(traj_time=5, points_num=10)
# print(cubic_traj)
