from singularity_loci import det_J
import numpy as np
import matplotlib.pyplot as plt
from max_workspace import workspace_optimization
import time 
# Kinematic parameters
Rb = 300 / np.sqrt(3)
l1 = 166
l2 = 150
Re = 30
param = [Rb, l1, l2, Re]

# we set orientation of the robot to 0 
# since the robot is symetrical, the motors are positioned 120 degres apart from each other then the singularities (espicially parralel ones) rotates with theta_e and we don't change the shape of the singularities 
theta_e = 0

# we break down our workspace into eight 400*400 matrix, one for each working mode (each motor has two working modes elbow up or down see ressources )
coord = np.linspace(-210,210,200)
det_J_p, det_J_s = det_J(param, coord, theta_e)        

loci = np.squeeze(abs(det_J_p[1, :, :]))
tm1 = time.time()
print(tm1)
max_limit = workspace_optimization(param,loci,0.5)
tm2 = time.time()
print(f"ur max is {max_limit} took {tm2-tm1}s ")