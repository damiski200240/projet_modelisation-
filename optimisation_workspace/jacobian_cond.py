# fonction qui sert à calculer le determinant et qui sera utilisé apres pour detecter si une configuration donne une singularité ou pas 


from numpy import cos , sin, array,pi
from numpy .linalg import cond



def jacondian_cond(alpha, beta, theta_e, param ) : 
    #alpha sont les angles moteurs 
    #beta sont les angles entre le premier et le deuxieme bras 
    #theta_e est l'orientation de la platforme "le triangle du milieu"
    #param sont les parametres du workspace, [Rb,L1,L2,Re]

    #extraire les parametres 
    alpha_1, alpha_2, alpha_3 = alpha[0], alpha[1], alpha[2]
    beta_1, beta_2, beta_3 = beta[0], beta[1], beta[2]
    L1 = param[1] # longuer premier bras
    Re = param[-1] # rayon de l'effecteur 

    # definition des deux matrices 

    # matrice serie B = J_s
    J_s = array([[L1*sin(beta_1-alpha_1), 0, 0], 
                [0,L1*sin(beta_2 - alpha_2),0],
                [0,0,L1*sin(beta_3-alpha_3)]])
    
    cond_J_s = cond(J_s)
    det_J_s = 1/cond_J_s


    # matrice parralle A = J_p
    J_p = array([[-cos(beta_1), -sin(beta_1), Re*sin(beta_1-theta_e-pi/6)
      -cos(beta_2), -sin(beta_2), Re*sin(beta_2-theta_e-5*pi/6)
      -cos(beta_3), -sin(beta_3), Re*sin(beta_3-theta_e-3*pi/2)]])
    
    cond_J_p = cond(J_p)
    det_J_p = 1/cond_J_p

    return det_J_s, det_J_p