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
    # Kinematic parameters
    K, L1, L2, R = param

    # Active joint angles
    th1, th2, th3 = theta

    # Passive joint angles
    ph1, ph2, ph3 = phi

    # Calculate Jacobians
    dJth = L1**3 * np.sin(ph1 - th1) * np.sin(ph2 - th2) * np.sin(ph3 - th3)

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
    TH1, TH2, TH3 = np.meshgrid(th1, th2, th3, indexing='ij')
    comb = np.stack((TH1.ravel(), TH2.ravel(), TH3.ravel()), axis=-1)

    # Initialize determinant arrays
    detJx = np.zeros(8)
    detJth = np.zeros(8)

    # Iterate through all combinations (working modes)
    for k in range(len(comb)):
        th1_k, th2_k, th3_k = comb[k]
        # Calculate the associated phi1, phi2, phi3
        ph1, ph2, ph3, psi1 , psi2 , psi3 = ikm_phi_psi(param, [th1_k, th2_k, th3_k], x, y, a)
        th = [th1_k, th2_k, th3_k]
        ph = [ph1, ph2, ph3]
        # Calculate determinants
        Jth, Jx = det_jacobian(param, th, ph, a)
        # Save determinant for the particular working mode
        detJth[k] = Jth
        detJx[k] = Jx

    return detJth, detJx

def get_best_mode(det_J_s):
    variations = [np.max(det_J_s[i]) - np.min(det_J_s[i]) for i in range(8)]
    best_index = np.argmax(variations)
    return best_index


def det_J(param, coord, theta_e) : 
        
        
    det_J_s = np.zeros((8,len(coord), len(coord)))

    det_J_p = np.zeros((8,len(coord), len(coord)))
    for i in range(len(coord)) : 
        for j in range(len(coord)) : 
            theta_up, theta_down = ikm(coord[i], coord[j], theta_e, param)
            # Choose elbow-up configuration (you can choose elbow-down if needed)
            th1, th2, th3 = theta_up

            if (np.isreal(th1) and np.isreal(th2) and  np.isreal(th3)  ) : # meaning that our x and y coordinates are reachable by our 3 robotics arms  
                [dJ_p, dJ_s] = combine_det(param,th1,th2,th3,coord[i],coord[j],theta_e)
                # save determinant values
                det_J_p[:, i, j] = dJ_p
                det_J_s[:, i, j] = dJ_s
                
                
                

    Mp = np.max(np.abs(det_J_p)) 
    det_J_p = det_J_p/Mp 

    Ms = np.max(np.abs(det_J_s)) 
    det_J_s = det_J_s/Ms 
    
    best_idx = get_best_mode(det_J_s)
    print(f"[INFO] Best working mode index based on variation: {best_idx}")

    return det_J_p, det_J_s, best_idx

