from singularity_loci import det_J
import numpy as np
import matplotlib.pyplot as plt
from max_joint_angle import workspace_optimization
import time 
# Kinematic parameters
Rb = 150
l1 = 98.09268142  
l2 = 98.09268142 
Re = 77.91650034
param = [Rb, l1, l2, Re]

# we set orientation of the robot to 0 
# since the robot is symetrical, the motors are positioned 120 degres apart from each other then the singularities (espicially parralel ones) rotates with theta_e and we don't change the shape of the singularities 
theta_e = 0

# we break down our workspace into eight 400*400 matrix, one for each working mode (each motor has two working modes elbow up or down see ressources )
coord = np.linspace(-210,210,100)
det_J_p, det_J_s, best_idx = det_J(param, coord, theta_e)
loci = np.squeeze(np.abs(det_J_s[best_idx]))
tm1 = time.time()
print(tm1)
max_limit = workspace_optimization(param,loci,0.0002)
tm2 = time.time()
print(f"Your max is {max_limit}, took {tm2 - tm1}s")

# Plot the final singularity loci
plt.figure()
plt.imshow(
    np.abs(loci),
    extent=[-210, 210, -210, 210],
    origin='lower',
    aspect='equal',
    cmap='viridis'
)
c = plt.colorbar()
c.set_label('Absolute value of det(J) (normalized)')
plt.title('Direct (type II) singularity contours')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')
plt.show()
