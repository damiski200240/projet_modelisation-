#this gives us the angles of the robots links , where phi is the angle between the first and second leg 
#and psi is the angles between the second leg and the platforme 
import numpy as np


def ikm_phi_psi(param,theta,x,y,theta_e ):
    #  where param are the parameteres of the robot 
    # theta are the angles of the motors 
    # x, y and theta_e are the position and orientation of the robot.
    K, l1, l2, R = param
    th1, th2, th3 = theta[0], theta[1], theta[2]
    xA = K * np.cos(np.pi / 6)
    yA = K * np.sin(np.pi / 6)
    cph1 = (xA - l1 * np.cos(th1) - R * np.cos(theta_e + np.pi / 6) + x) / l2
    sph1 = (yA - l1 * np.sin(th1) - R * np.sin(theta_e + np.pi / 6) + y) / l2
    cph2 = (-xA - l1 * np.cos(th2) - R * np.cos(theta_e + 5 * np.pi / 6) + x) / l2
    sph2 = (yA - l1 * np.sin(th2) - R * np.sin(theta_e + 5 * np.pi / 6) + y) / l2
    cph3 = (-(l1 * np.cos(th3) + R * np.cos(theta_e - np.pi / 2) - x)) / l2
    sph3 = (-(K + l1 * np.sin(th3) + R * np.sin(theta_e - np.pi / 2) - y)) / l2
    ph1 = np.arctan2(sph1, cph1)
    ph2 = np.arctan2(sph2, cph2)
    ph3 = np.arctan2(sph3, cph3)
    psi1 = np.pi - ph1 + np.pi / 6 + theta_e
    psi2 = np.pi - ph2 + 5 * np.pi / 6 + theta_e
    psi3 = np.pi - ph3 + 3 * np.pi / 2 + theta_e
    return ph1, ph2, ph3, psi1, psi2, psi3