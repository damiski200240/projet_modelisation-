#getting all the singularities in a workspace 
#the goal is to divied the workspace space into a mesh grid or a matrix 
#where for each point  will be calculated the jacobian of both matrices A and B
#for parallel singularities type I 
#and serial singularity type II

import numpy as np
from ikm import ikm 
from ikm_phi_psi import ikm_phi_psi
import matplotlib.pyplot as plt

# Kinematic parameters
Rb = 300 / np.sqrt(3)
l1 = 166
l2 = 160
Re = 30
param = [Rb, l1, l2, Re]

# we set orientation of the robot to 0 
# since the robot is symetrical, the motors are positioned 120 degres apart from each other then the singularities (espicially parralel ones) rotates with theta_e and we don't change the shape of the singularities 
theta_e = 0

# we break down our workspace into eight 400*400 matrix, one for each working mode (each motor has two working modes elbow up or down see ressources )
coord = np.linspace(-210,210,400)



def det_jacobian(param, theta, phi, alpha):
    """
    Calculates the determinants of the Jacobians for the given parameters.

    Args:
        param: List of kinematic parameters [K, L1, L2, R].
        theta: List of active joint angles [th1, th2, th3].
        phi: List of passive joint angles [ph1, ph2, ph3].
        alpha: Orientation angle.

    Returns:
        dJth: Determinant of the Jacobian with respect to theta.
        dJx: Determinant of the Jacobian with respect to x.
    """
    K, L1, L2, R = param
    th1, th2, th3 = theta
    ph1, ph2, ph3 = phi
    dJth = L1 ** 3 * np.sin(ph1 - th1) * np.sin(ph2 - th2) * np.sin(ph3 - th3)
    Jx = np.array([
        [-np.cos(ph1), -np.sin(ph1), R * np.sin(ph1 - alpha - np.pi / 6)],
        [-np.cos(ph2), -np.sin(ph2), R * np.sin(ph2 - alpha - 5 * np.pi / 6)],
        [-np.cos(ph3), -np.sin(ph3), R * np.sin(ph3 - alpha - 3 * np.pi / 2)]
    ])
    dJx = np.linalg.det(Jx)
    return dJth, dJx

def combine_det(param, th1, th2, th3, x, y, a):
    """
    Combines the input angles theta1, theta2, theta3 into the 8 possible working modes
    and calculates the determinants for each working mode.

    Args:
        param: List of kinematic parameters [Rb, l1, l2, Re].
        th1, th2, th3: Input angles (can be arrays).
        x, y, a: Position and orientation of the end effector.

    Returns:
        detJth: Determinants of the Jacobian with respect to theta.
        detJx: Determinants of the Jacobian with respect to x.
    """
    # Generate all combinations of th1, th2, th3
    from itertools import product
    th_combos = np.array(list(product(th1, th2, th3)))
    detJx = np.zeros(8)
    detJth = np.zeros(8)
    for k, th in enumerate(th_combos):
        if np.any(np.isnan(th)):
            detJth[k] = np.nan
            detJx[k] = np.nan
            continue
        ph1, ph2, ph3, *_ = ikm_phi_psi(param, th, x, y, a)
        ph = [ph1, ph2, ph3]
        if np.any(np.isnan(ph)):
            detJth[k] = np.nan
            detJx[k] = np.nan
            continue
        Jth, Jx = det_jacobian(param, th, ph, a)
        if np.isnan(Jth) or np.isnan(Jx):
            detJth[k] = np.nan
            detJx[k] = np.nan
        else:
            detJth[k] = Jth
            detJx[k] = Jx
            
    return detJth, detJx



def det_J(param, coord, theta_e, nb_points) :  
    valid_points = 0 
    det_J_s = np.zeros((8,nb_points, nb_points))
    det_J_p = np.zeros((8,nb_points, nb_points))
    for j in range(nb_points) : 
        for i in range(nb_points) : 
            th1, th2, th3 = ikm(coord[j], coord[i], theta_e, param)
            if (np.isreal(th1).all() and np.isreal(th2).all() and  np.isreal(th3).all()  ) : # meaning that our x and y coordinates are reachable by our 3 robotics arms  
                [dJ_p, dJ_s] = combine_det(param,th1,th2,th3,coord[j],coord[i],theta_e)
                # save determinant values
                det_J_p[:, i, j] = dJ_p
                det_J_s[:, i, j] = dJ_s
                if not np.all(np.isnan(dJ_s)):
                    valid_points += 1
    print(f"Number of valid points: {valid_points}")        
                
    Ms = np.nanmax(np.abs(det_J_s))
    det_J_s = det_J_s / Ms 
    Mp = np.max(np.abs(det_J_p))
    det_J_p = det_J_p/Mp 

    return det_J_p, det_J_s
