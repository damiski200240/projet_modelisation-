# methode pour calucler le modele geometrique inverse d'une maniere qui donne les solutions rapidement avec le coude bas et haut. Ceci sera couplé avec jacobien_cond pour trouver les singularités 
# et de enfin trouver les params qui maximise le workspace
# 

from numpy import sqrt, sin , cos, pi, arctan


def ikm(x, y, theta_e, param):
    Rb = param[0]
    L1 = param[1]
    L2 = param[2]
    Re = param[3]

    xO, yO = [], []
    xE, yE = [], []
    a_list, b_list, c_list = [], [], []

    for i in range(3):  # i = 0, 1, 2
        angle_O = 2 * i * pi / 3 + pi / 2
        angle_E = theta_e + 2 * i * pi / 3 - pi / 2

        xO_i = Rb * cos(angle_O)
        yO_i = Rb * sin(angle_O)
        xE_i = Re * cos(angle_E)
        yE_i = Re * sin(angle_E)

        xO.append(xO_i)
        yO.append(yO_i)
        xE.append(xE_i)
        yE.append(yE_i)

    for i in range(3):
        a = Rb**2 + Re**2 + L1**2 - L2**2 + x**2 + y**2 + 2*xO[i]*xE[i] - 2*xO[i]*x - 2*xE[i]*x + 2*yO[i]*yE[i] - 2*yO[i]*y - 2*yE[i]*y
        b = 2 * L1 * (xO[i] + xE[i] - x)
        c = 2 * L1 * (yO[i] + yE[i] - y)

        a_list.append(a)
        b_list.append(b)
        c_list.append(c)
    
    
    
    theta_up = []
    theta_down = []
    for i in range(0 , 3) : 
       delta = max(0, sqrt(c_list[i] ** 2 + b_list[i]**2 - a_list[i] ** 2) )
       th_up = 2 * arctan((-c_list[i] + delta) / a_list[i] - b_list[i])
       th_down = 2 * arctan((-c_list[i] - delta) / a_list[i] - b_list[i])
       theta_up.append(th_up)
       theta_down.append(th_down)
    
    return theta_up , theta_down
    