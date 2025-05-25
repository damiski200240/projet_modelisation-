
from get_compliant_workspace import get_compliant_workspace
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi , cos, sin 
# Kinematic parameters
Rb = 150
l1 = 98.18861278  
l2 = 98.18861278
Re = 77.89302838
param = [Rb, l1, l2, Re]

# Compliant joint limitation
limit = 300* np.pi / 180

# Working mode
mode = '+ + +'

# Home position
home_pos = [0, 0, 0]

# Orientation of the end-effector
orientation = 0 * np.pi / 180

# Obtain the compliant workspace
comp_workspace = get_compliant_workspace(param, limit, home_pos, mode, orientation,plot=True)
print(f"area : {comp_workspace.area}mm²")
# Plot the workspace
plt.figure()
x, y = comp_workspace.exterior.xy
plt.plot(x, y, label="Compliant workspace", color='black', linewidth=2)
plt.axis('equal')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')
plt.title("Compliant workspace")
plt.show()