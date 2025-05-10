#this gives us the angles of the robots links , where phi is the angle between the first and second leg 
#and psi is the angles between the second leg and the platforme 
import numpy as np


def ikm_phi_psi(param,theta,x,y,theta_e ):
    #  where param are the parameteres of the robot 
    # theta are the angles of the motors 
    # x, y and theta_e are the position and orientation of the robot.
    
    Rb, L1, L2, Re = param
    
     # Extract theta123
    th1 = theta[0]
    th2 = theta[1]
    th3 = theta[2]


    # Actuator position
    xA = Rb * np.cos(np.pi / 6)
    yA = Rb * np.sin(np.pi / 6)

    # Define cos(phi) and sin(phi)
    cph1 = (xA - L1 * np.cos(th1) - Rb * np.cos(theta_e + np.pi / 6) + x) / L2
    sph1 = (yA - L1 * np.sin(th1) - Rb * np.sin(theta_e + np.pi / 6) + y) / L2
    cph2 = (-xA - L1 * np.cos(th2) - Rb * np.cos(theta_e + 5 * np.pi / 6) + x) / L2
    sph2 = (yA - L1 * np.sin(th2) - Rb * np.sin(theta_e + 5 * np.pi / 6) + y) / L2
    cph3 = (-(L1 * np.cos(th3) + Rb * np.cos(theta_e - np.pi / 2) - x)) / L2
    sph3 = (-(Rb + L1 * np.sin(th3) + Rb * np.sin(theta_e - np.pi / 2) - y)) / L2

    # Solve for phi using arctan2
    ph1 = np.arctan2(sph1, cph1)
    ph2 = np.arctan2(sph2, cph2)
    ph3 = np.arctan2(sph3, cph3)

    # Solve for psi
    psi1 = np.pi - ph1 + np.pi / 6 + theta_e
    psi2 = np.pi - ph2 + 5 * np.pi / 6 + theta_e
    psi3 = np.pi - ph3 + 3 * np.pi / 2 + theta_e

    return ph1, ph2, ph3, psi1, psi2, psi3