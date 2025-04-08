import numpy as np

global L1, L2, Rb, Re

def solve_eq_NL(q,eff): 
    q = [q0,q1,q2, q3, q4 , q5, q6]
    alpha = [alpha1, alpha2 , alpha3]
    beta =[beta1, beta2,beta3]

    for i in range(len(q)) : 
        if i % 2 == 0 : 
            q[i] = beta[i]
        else : 
            q[i] = alpha[i]

    ang1=[0 2*pi/3 4*pi/3];


    ang2=[-pi/2 pi/6 5*pi/6];

    RotEff = np.array([[np.cos(eff[2]), -np.sin(eff[2])],
                        [np.sin(eff[2]), np.cos(eff[2])]])
    Transl = np.array([eff[0], eff[1]])
    THEff = np.block([[RotEff, Transl.reshape(2, 1)], [0, 0, 1]])
    

    F = []


    for i in range(3) : 
        PEi_E = np.array([Re * np.cos(ang2[i]), Re * np.sin(ang2[i]), 1])

        PEi_0 = np.dot(THEff, PEi_E)


        Rot = np.array([[np.cos(ang1[i]), -np.sin(ang1[i])],
                        [np.sin(ang1[i]), np.cos(ang1[i])]])
        THRi_0 = np.block([[Rot, np.array([[Rb * np.cos(ang2[i])], [Rb * np.sin(ang2[i])]])], [0, 0, 1]])

        PBi = np.dot(THRi_0, np.array([
            L1 * np.cos(alpha[i]) + L2 * np.cos(alpha[i] + beta[i]),
            L1 * np.sin(alpha[i]) + L2 * np.sin(alpha[i] + beta[i]),
            1
        ]))

        F.append(PBi[0] - PEi_0[0])
        F.append(PBi[1] - PEi_0[1])

    return F