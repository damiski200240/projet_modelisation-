# the logic behind this code 
# we get the reachable area of each motor indivdually 
# the overlap of these 3 areas are the compliant workspace of our end effector 
# we will try to maximize this workspace after 

import numpy as np
from numpy import linspace, pi, cos,sin
import re
from ikm import ikm
from ikm_phi_psi import ikm_phi_psi
from scipy.spatial import ConvexHull
from shapely import Polygon
import matplotlib.pyplot as plt
from get_coord import get_coord


def get_compliant_workspace(param, limit, home_pos, mode, orientation) : 
    # params are the parameteres of the workspace 
    # limit is the limit angle of each motor
    # mode is either elbow up (+) or down (-) for each robot : see the image in ressources  
    # orientation is the orientation of the end effector 
    # home pos is the starting posiotion if the end effector
    x_init = home_pos[0]
    y_init = home_pos[1]
    theta_init = home_pos[2]
    Rb = param[0]
    L1 = param[1]
    L2 = param[2]
    Re = param[3]

    # Convert "+ + +" to [1, 1, 1] and "- + -" to [2, 1, 2]
    mode = [1 if m == '+' else 2 for m in mode.strip().split()]
    wm1, wm2, wm3 = mode

    # Initial joint angles
    theta_up, theta_down = ikm(x_init, y_init, theta_init, param)
    theta_10 = theta_up[0] if wm1 == 1 else theta_down[0]
    theta_20 = theta_up[1] if wm2 == 1 else theta_down[1]
    theta_30 = theta_up[2] if wm3 == 1 else theta_down[2]

    [ph10, ph20, ph30, psi10, psi20, psi30] = ikm_phi_psi(param, [theta_10, theta_20, theta_30], x_init, y_init, theta_init)

    # calculating the reachable area of each robot
    # for the first leg

    # sweep across all the possible values of theta and phi, initial value +- limit
    theta = linspace(theta_10 - limit, theta_10 + limit)
    phi = linspace(ph10 - limit, ph10 + limit)

    # calculate psi from phi: psi = pi - phi + OFFSET + alpha + orientation
    psi = pi - phi + pi/6 + theta_init + orientation

    # find the positions where the joint limit is not respected
    valid_idx = np.abs(psi - psi10) < limit

    # keep only valid phi values
    phi = phi[valid_idx]

    cord1 = []
    for t in theta:
        for p in phi:
            x, y = get_coord(param, t, p, theta_init + orientation, 2*pi/3)
            cord1.append([x, y])

    # for the second leg

    # sweep across all the possible values of theta and phi, initial value +- limit
    theta = linspace(theta_20 - limit, theta_20 + limit)
    phi = linspace(ph20 - limit, ph20 + limit)

    # calculate psi from phi: psi = pi - phi + OFFSET + alpha + orientation
    psi = pi - phi + 5*pi/6 + theta_init + orientation

    # find the positions where the joint limit is not respected
    valid_idx = np.abs(psi - psi20) < limit

    # keep only valid phi values
    phi = phi[valid_idx]

    cord2 = []
    for t in theta:
        for p in phi:
            x, y = get_coord(param, t, p, theta_init + orientation, -2*pi/3)
            cord2.append([x, y])

    # for the third leg

    # sweep across all the possible values of theta and phi, initial value +- limit
    theta = linspace(theta_30 - limit, theta_30 + limit)
    phi = linspace(ph30 - limit, ph30 + limit)

    # calculate psi from phi: psi = pi - phi + OFFSET + alpha + orientation
    psi = pi - phi + 3*pi/2 + theta_init + orientation

    # find the positions where the joint limit is not respected
    valid_idx = np.abs(psi - psi30) < limit

    # keep only valid phi values
    phi = phi[valid_idx]

    cord3 = []
    for t in theta:
        for p in phi:
            x, y = get_coord(param, t, p, theta_init + orientation, 0)
            cord3.append([x, y])
        
        
    # plot the worksapce
    coord1 = np.array(cord1)
    hull = ConvexHull(coord1)
    k = hull.vertices
    wrkspace1 = Polygon(coord1[k])
    
    coord2 = np.array(cord2)
    hull = ConvexHull(coord2)
    k = hull.vertices
    wrkspace2 = Polygon(coord2[k])
    
    
    coord3 = np.array(cord3)
    hull = ConvexHull(coord3)
    k = hull.vertices
    wrkspace3 = Polygon(coord3[k])
    xO, yO = [], []

    # get the position of the motors 
    for i in range(3):  # i = 0, 1, 2
            angle_O = 2 * i * pi / 3 + pi / 2
            xO_i = Rb * cos(angle_O)
            yO_i = Rb * sin(angle_O)
            xO.append(xO_i)
            yO.append(yO_i)

    # plt.figure()
    # plt.plot(*wrkspace1.exterior.xy, label='1st kinematic chain')
    # plt.plot(*wrkspace2.exterior.xy, label='2nd kinematic chain')
    # plt.plot(*wrkspace3.exterior.xy, label='3rd kinematic chain')
    # plt.plot(xO, yO, 'ro', label="Moteurs (O1, O2, O3)")  # points rouges 
    # plt.legend()
    # plt.axis('equal')
    # plt.show()
    
    comp_workspace = wrkspace1.intersection(wrkspace2).intersection(wrkspace3)
    # print(f"area if our workspace is {comp_workspace.area*1e-6} m²")
    
    return comp_workspace
    
    