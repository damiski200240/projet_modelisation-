from singularity_loci import det_J
import numpy as np
import matplotlib.pyplot as plt
from max_joint_angle import workspace_optimization
import time 
from get_compliant_workspace import get_compliant_workspace
from math import *
from ikm import ikm 
from singularity_loci import combine_det

nb_points = 100
alpha = 0
K = 150
l1 = 98.18861278
l2 = 98.18861278
R = 77.89302838
param = [K, l1, l2, R]
limit = 300 * np.pi / 180
mode = '+ + +'
home_pos = [0, 0, 0]
orientation = 0 * np.pi / 180
valid_points = 0

coord = np.linspace(-210, 210, nb_points)
detJth = np.zeros((8, nb_points, nb_points))
detJx = np.zeros((8, nb_points, nb_points))

for j in range(nb_points):
    for k in range(nb_points):
        th1, th2, th3 = ikm(param, coord[j], coord[k], alpha)
        if (np.isreal(th1).all() and np.isreal(th2).all() and np.isreal(th3).all()):
            dJth, dJx = combine_det(param, th1, th2, th3, coord[j], coord[k], alpha)
            detJth[:, k, j] = dJth
            detJx[:, k, j] = dJx
            if not np.all(np.isnan(dJx)):
                valid_points += 1
print(f"Number of valid points: {valid_points}")

Mx = np.nanmax(np.abs(detJx))

detJx /= Mx
Mth = np.nanmax(np.abs(detJth))
detJth /= Mth
poly = get_compliant_workspace(param, 300* np.pi / 180, [0,0,0], '+ + +', 0)
x, y = poly.exterior.xy

plt.figure()
plt.imshow(np.abs(detJx[0]), extent=[-210, 210, -210, 210], origin='lower')
plt.colorbar(label='Absolute value of det(Jx) (normalized)')
plt.plot(x, y, color='red', linewidth=1.5, label="Compliant workspace")
plt.title('Direct (type II) singularity contours')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')
plt.axis('equal')
plt.show()

plt.figure()
plt.imshow(np.abs(detJth[0]), extent=[-210, 210, -210, 210], origin='lower')
plt.colorbar(label='Absolute value of det(Jth) (normalized)')
plt.plot(x, y, color='red', linewidth=1.5, label="Compliant workspace")
plt.title('Direct (type I) singularity contours')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')
plt.axis('equal')
plt.show()


loci = np.squeeze(np.abs(detJx[0,:,:]))
tm1 = time.time()
print(tm1)
max_limit = workspace_optimization(param,loci,0.5,210)
tm2 = time.time()
print(f"Your max is {max_limit}, took {tm2 - tm1}s")
