
from get_compliant_workspace import get_compliant_workspace
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi , cos, sin 
# Kinematic parameters
Rb = 300 / np.sqrt(3)
l1 = 166
l2 = 160
Re = 30
param = [Rb, l1, l2, Re]

# Compliant joint limitation
limit = 80* np.pi / 180

# Working mode
mode = '- - -'

# Home position
home_pos = [0, 0, 0]

# Orientation of the end-effector
orientation = 0 * np.pi / 180

# Obtain the compliant workspace
comp_workspace = get_compliant_workspace(param, limit, home_pos, mode, orientation)

# Plot the workspace
plt.figure()
x, y = comp_workspace.exterior.xy
plt.plot(x, y, label="Compliant workspace", color='black', linewidth=2)
plt.axis('equal')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')
plt.title("Compliant workspace")
plt.show()