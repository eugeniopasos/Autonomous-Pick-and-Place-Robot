import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))
from Robot import Robot

import numpy  as np
robot = Robot()

# Print 3 test cases for forward kinematic transformations
print("Tbase-ee for [0, 0, 0, 0]")
print(robot.get_fk(np.radians([0, 0, 0, 0])))
print()

print("Tbase-ee for [15, -45, -60, 90]")
print(robot.get_fk(np.radians([15, -45, -60, 90])))
print()

print("Tbase-ee for [-90, 15, 30, -45]")
print(robot.get_fk(np.radians([-90, 15, 30, -45])))