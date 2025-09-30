# (c) 2024 S. Farzan, Electrical Engineering Department, Cal Poly
# Skeleton Robot class for OpenManipulator-X Robot for EE 471

import numpy as np
from OM_X_arm import OM_X_arm
from DX_XM430_W350 import DX_XM430_W350

"""
Robot class for controlling the OpenManipulator-X Robot.
Inherits from OM_X_arm and provides methods specific to the robot's operation.
"""
class Robot(OM_X_arm):
    """
    Initialize the Robot class.
    Creates constants and connects via serial. Sets default mode and state.
    """
    def __init__(self):
        super().__init__()

        # Robot Dimensions (in mm)
        self.mDim = [77, 130, 124, 126]
        self.mOtherDim = [128, 24]
        
        # Set default mode and state
        # Change robot to position mode with torque enabled by default
        # Feel free to change this as desired
        self.write_mode('position')
        self.write_motor_state(True)

        # Set the robot to move between positions with a 5 second trajectory profile
        # change here or call writeTime in scripts to change
        self.write_time(5)

    """
    Sends the joints to the desired angles.
    Parameters:
    goals (list of 1x4 float): Angles (degrees) for each of the joints to go to.
    """
    def write_joints(self, goals):
        goals = [round(goal * DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET) % DX_XM430_W350.TICKS_PER_ROT for goal in goals]
        self.bulk_read_write(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals)

    """
    Creates a time-based profile (trapezoidal) based on the desired times.
    This will cause write_position to take the desired number of seconds to reach the setpoint.
    Parameters:
    time (float): Total profile time in seconds. If 0, the profile will be disabled (be extra careful).
    acc_time (float, optional): Total acceleration time for ramp up and ramp down (individually, not combined). Defaults to time/3.
    """
    def write_time(self, time, acc_time=None):
        if acc_time is None:
            acc_time = time / 3

        time_ms = int(time * DX_XM430_W350.MS_PER_S)
        acc_time_ms = int(acc_time * DX_XM430_W350.MS_PER_S)

        self.bulk_read_write(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, [acc_time_ms]*self.motorsNum)
        self.bulk_read_write(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, [time_ms]*self.motorsNum)

    """
    Sets the gripper to be open or closed.
    Parameters:
    open (bool): True to set the gripper to open, False to close.
    """
    def write_gripper(self, open):
        if open:
            self.gripper.write_position(-45)
        else:
            self.gripper.write_position(45)

    def read_gripper(self):
        if open:
            pos = self.gripper.read_position()
        else:
            pos = self.gripper.read_position()
        return pos

    """
    Sets position holding for the joints on or off.
    Parameters:
    enable (bool): True to enable torque to hold the last set position for all joints, False to disable.
    """
    def write_motor_state(self, enable):
        state = 1 if enable else 0
        states = [state] * self.motorsNum  # Repeat the state for each motor
        self.bulk_read_write(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, states)

    """
    Supplies the joints with the desired currents.
    Parameters:
    currents (list of 1x4 float): Currents (mA) for each of the joints to be supplied.
    """
    def write_currents(self, currents):
        current_in_ticks = [round(current * DX_XM430_W350.TICKS_PER_mA) for current in currents]
        self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, current_in_ticks)

    """
    Change the operating mode for all joints.
    Parameters:
    mode (str): New operating mode for all joints. Options include:
        "current": Current Control Mode (writeCurrent)
        "velocity": Velocity Control Mode (writeVelocity)
        "position": Position Control Mode (writePosition)
        "ext position": Extended Position Control Mode
        "curr position": Current-based Position Control Mode
        "pwm voltage": PWM Control Mode
    """
    def write_mode(self, mode):
        if mode in ['current', 'c']:
            write_mode = DX_XM430_W350.CURR_CNTR_MD
        elif mode in ['velocity', 'v']:
            write_mode = DX_XM430_W350.VEL_CNTR_MD
        elif mode in ['position', 'p']:
            write_mode = DX_XM430_W350.POS_CNTR_MD
        elif mode in ['ext position', 'ep']:
            write_mode = DX_XM430_W350.EXT_POS_CNTR_MD
        elif mode in ['curr position', 'cp']:
            write_mode = DX_XM430_W350.CURR_POS_CNTR_MD
        elif mode in ['pwm voltage', 'pwm']:
            write_mode = DX_XM430_W350.PWM_CNTR_MD
        else:
            raise ValueError(f"writeMode input cannot be '{mode}'. See implementation in DX_XM430_W350 class.")

        self.write_motor_state(False)
        write_modes = [write_mode] * self.motorsNum  # Create a list with the mode value for each motor
        self.bulk_read_write(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, write_modes)
        self.write_motor_state(True)

    """
    Gets the current joint positions, velocities, and currents.
    Returns:
    numpy.ndarray: A 3x4 array containing the joints' positions (deg), velocities (deg/s), and currents (mA).
    """
    def get_joints_readings(self):
        readings = np.zeros((3, 4))
        
        positions = np.array(self.bulk_read_write(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION))
        velocities = np.array(self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY))
        currents = np.array(self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT))

        # Take two's complement of velocity and current data
        for i in range(4):
            if velocities[i] > 0x7fffffff:
                velocities[i] = velocities[i] - 4294967296
            if currents[i] > 0x7fff:
                currents[i] = currents[i] - 65536

        readings[0, :] = (positions - DX_XM430_W350.TICK_POS_OFFSET) / DX_XM430_W350.TICKS_PER_DEG
        readings[1, :] = velocities / DX_XM430_W350.TICKS_PER_ANGVEL
        readings[2, :] = currents / DX_XM430_W350.TICKS_PER_mA

        return readings

    """
    Sends the joints to the desired velocities.
    Parameters:
    vels (list of 1x4 float): Angular velocities (deg/s) for each of the joints to go at.
    """
    def write_velocities(self, vels):
        vels = [round(vel * DX_XM430_W350.TICKS_PER_ANGVEL) for vel in vels]
        self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels)

    
    
    # ############ #
    # Eugenio Code #
    # ############ #

    # ##### #
    # Lab 2 #
    # ##### #

    """
    get_dh_row_mat(arr):

    Input: a 1 x 4 array corresponding to a row of the DH parameter table for a given joint.
    Output: a 4 x 4 numpy array representing the homogeneous transformation matrix Ai .
    Description: Calculates the intermediate transformation Ai for a DH parameter table row
    
    """
    
    def get_dh_row_mat(self, arr):
        return np.array([[np.cos(arr[0]),  -np.sin(arr[0])*np.cos(arr[3]),    np.sin(arr[0])*np.sin(arr[3]),         arr[2]*np.cos(arr[0])],
                         [np.sin(arr[0]),   np.cos(arr[0])*np.cos(arr[3]),   -np.cos(arr[0])*np.sin(arr[3]),         arr[2]*np.sin(arr[0])],
                         [0,                np.sin(arr[3]),                   np.cos(arr[3]),                        arr[1]               ],
                         [0,                0,                                0,                                     1                    ]])

    """
    get_int_mat(j):

    Input: a 1 x 4 numpy array specifying the joint angles.
    Output: a 4 x 4 x 4 numpy array of A matrices for specified joint angles (A1; A2; A3; A4).
    Description: Utilizing get_dh_row_mat(), this method uses the DH table and joint
    angles to calculate the intermediate transformation Ai for all rows at specific joint angles.
    """
    def get_int_mat(self, j):
        return np.array([self.get_dh_row_mat([j[0],                               77,   0, -np.pi/2]),
                         self.get_dh_row_mat([j[1]-(np.pi/2 -np.arcsin(24/130)),   0, 130,        0]),
                         self.get_dh_row_mat([j[2]+(np.pi/2 -np.arcsin(24/130)),   0, 124,        0]),
                         self.get_dh_row_mat([j[3],                                0, 126,        0]),])

    """
    get_acc_mat(j):

    Input: a 1 x 4 numpy array specifying the joint angles.
    Output: a 4 x 4 x 4 numpy array of T matrices (transforms from joint i to the base) for
    specified joint angles (T01 ; T02 ; T 03 ; T 04 ).
    Description: Using the A matrices from get_int_mat(), this method calculates the
    accumulative transformations for all joints at specific joint angles.
    """

    def get_acc_mat(self, j):
        a = self.get_int_mat(j)
        return np.array([a[0],
                        a[0] @ a[1],
                        a[0] @ a[1] @ a[2],
                        a[0] @ a[1] @ a[2] @ a[3]])
    
    """
    get_fk(j):

    Input: a 1 x 4 numpy array specifying the joint angles.
    Output: a 4 x 4 numpy array representing the end-effector to base transformation (T 04)
    Description: Using the A matrices from get_int_mat(), this method calculates the
    forward kinematics of the robot as a 4 x 4 homogeneous transformation matrix repre-
    senting the position and orientation of the end-effector frame with respect to the base
    frame (i.e. Tbase-ee )
    """
    
    def get_fk(self, j):
        a = self.get_int_mat(j)
        return a[0] @ a[1] @ a[2] @ a[3]
    
    """
    get_current_fk()

    Input: none
    Output: 4 x 4 numpy array representing the end-effector to base transformation for
    current joint angles
    Description: Retrieves the last read joint angles using get_joints_readings() and
    calculates the end-effector to base transformation using get_fk()
    """

    def get_current_fk(self):
        return self.get_fk(np.radians((self.get_joints_readings()[0])))
    
    """
    Input: a 1 x 4 numpy array specifying the joint angles
    Output: a 1 x 5 numpy array containing the end-effector position (x, y , z in mm) and
    orientation (pitch and yaw in degrees)
    Description: Calculates the end-effector position and orientation for given joint angles. It
    returns the x, y , z coordinates in millimeters and the pitch & yaw in degrees with respect
    to the base frame.
    """
    def get_ee_pos(self, j):
        a = self.get_current_fk()
        return np.array([a[0][3], a[1][3], a[2][3], np.rad2deg(-j[1] - j[2] - j[3]), np.rad2deg(j[0])])
    
    # ##### #
    # Lab 3 #
    # ##### #

    """
    Input: a 1x4 numpy array representing the desired end-effector pose (x; y ; z;  ̧), where
    x; y ; z are in millimeters, and  ̧ is the pitch orientation angle in degrees.
    Output: a 1 x 4 numpy array for corresponding joint angles in degrees (q1; q2; q3; q4).
    Description: Calculates the inverse kinematics to determine the joint angles for the spec-
    ified end-effector pose in task space.
    *The method should calculate two solutions (elbow-up and elbow-down) when possible,
    and then return the elbow-up as the output.
    *If the pose is unreachable or no valid configuration is found, the method should raise
    a ValueError1 with an appropriate message.
    """
    def get_ik(self, p):
        l1 = 77
        l2 = 130
        l3 = 124
        l4 = 126
        l21 = 128
        l22 = 24
        r = np.sqrt(p[0]**2 + p[1]**2)
        rw = r - l4*np.cos(np.radians(p[3]))
        zw = p[2] - l1 - l4*np.sin(np.radians(p[3]))
        dw = np.sqrt(rw**2 + zw**2)

        u = np.arctan2(zw, rw)
        cosb = (l2**2 + l3**2 - dw**2) / (2*l2*l3)
        sinb = np.sqrt(1 - cosb**2)
        cosg = (dw**2 + l2**2 - l3**2) / (2*dw*l2)
        sing = np.sqrt(1 - cosg**2)
        
        b = np.arctan2(sinb, cosb)
        g = np.arctan2(sing, cosg)
        d = np.arctan2(l22, l21)

        bb = np.arctan2(-sinb, cosb)
        gg = np.arctan2(-sing, cosg)
        
        # Solution 1 (Elbow Up)
        t1 = np.arctan2(p[1], p[0])
        t2 = np.pi/2 - d - g - u
        t3 = np.pi/2 + d - b
        t4 = -np.radians(p[3]) - t2 - t3

        # Solution 2 (Elbow Down)
        tt1 = np.arctan2(p[1], p[0])
        tt2 = np.pi/2 - d - gg- u
        tt3 = np.pi/2 + d - bb
        tt4 = -np.radians(p[3]) - tt2 - tt3

        if (-180 < np.rad2deg(t1) < 180) and (-115 < np.rad2deg(t2) < 90) and (-90 < np.rad2deg(t3) < 88) and (-100 < np.rad2deg(t4) < 115):
            return [t1, t2, t3, t4] 
        if (-180 < np.rad2deg(tt1) < 180) and (-115 < np.rad2deg(tt2) < 90) and (-90 < np.rad2deg(tt3) < 88) and (-100 < np.rad2deg(tt4) < 115):
            return [tt1, tt2, tt3, tt4]
        else:
            raise ValueError("Joint solutions not in range")
    
    # ##### #
    # Lab 5 #
    # ##### #
    
    """
    get_jacobian()
    Input: Joint configuration vector q ∈ R4 representing current joint angles [q1; q2; q3; q4]
    Output: Manipulator Jacobian matrix J(q) ∈ R6x4
    Description: Computes the manipulator Jacobian at the given configuration q
    """
    def get_jacobian(self, j):
        acc_mats = self.get_acc_mat(j)  

        o1 = acc_mats[0][:3, 3]  # Position of frame 1
        o2 = acc_mats[1][:3, 3]  # Position of frame 2
        o3 = acc_mats[2][:3, 3]  # Position of frame 3
        o4 = acc_mats[3][:3, 3]  # Position of frame 3


        z0 = [0,0,1]
        z1 = acc_mats[0][:3, 2]  
        z2 = acc_mats[1][:3, 2]  
        z3 = acc_mats[2][:3, 2]  

        Jv0 = np.cross(z0, o4)
        Jv1 = np.cross(z1, o4 - o1)
        Jv2 = np.cross(z2, o4 - o2)
        Jv3 = np.cross(z3, o4 - o3)

        # Combine the linear (Jv) and angular (Jw) components
        Jv = np.column_stack(np.radians((Jv0, Jv1, Jv2, Jv3)))
        Jw = np.column_stack((z0, z1, z2, z3))

        return np.vstack((Jv, Jw))
    """
    get_forward_diff_kinematics()
    Input: Joint angle vector q and joint velocity vector  ̇q
    Output: Task-space velocity vector  ̇p ∈ R6 containing the linear velocity components v
    and angular velocity components !
    Description: Computes instantaneous task-space velocities using the relationship  ̇p =
    J(q)  ̇q
    """
    def get_forward_diff_kinematics(self, j, jdot):
        J = self.get_jacobian(j)
        return J @ jdot


