# gives the position of the end effector 
# it builds the chain of vectors OOi, OiAi , AiBi , BiP see the image 3RRR_robot in ressources, with i = {1,2,3} and P the center of the effector
# We assume that O1 is mounted at the top and that O2 and O3 are mounted with the following angles in respect of O1: 2pi/3 and -2pi/3

from numpy import cos, sin
import numpy as np


def get_coord(param,theta,phi,theta_e,rot) : 
    Rb, L1, L2, Re = param
    
    rot_mat = [ [cos(rot), -sin(rot)],
                [sin(rot), cos(rot)] ]
    OA = np.dot(rot_mat, np.array([0, Rb]))
    AB = np.array([L1 * cos(theta), L1 * sin(theta)])
    BC = np.array([L2 * cos(phi), L2 * sin(phi)])
    CP = np.dot(rot_mat, np.array([Re * cos(theta_e- np.pi / 2), Re * sin(theta_e- np.pi / 2)]))

    # end-effector's final position
    end_effector = OA + AB + BC + CP

    return end_effector
