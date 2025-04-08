import numpy as np

# Variables globales à définir avant appel
global L1, L2, Rb, Re

L1=0.10  #long segment 1
L2=0.10  #long segment 2
Rb=0.1322594  # Rayon base
Re=0.07 # Rayon effecteur

def solve_eq_NL(q, eff):
    global L1, L2, Rb, Re

    alpha = [q[0], q[2], q[4]]
    beta  = [q[1], q[3], q[5]]

    ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]
    ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

    # Transformation homogène de l'effecteur
    RotEff = np.array([
        [np.cos(eff[2]), -np.sin(eff[2])],
        [np.sin(eff[2]),  np.cos(eff[2])]
    ])
    Transl = np.array([[eff[0]], [eff[1]]])
    THEff = np.vstack((
        np.hstack((RotEff, Transl)),  # 2x3
        np.array([[0, 0, 1]])         # 1x3
    ))

    F = []

    for i in range(3):
        # Position de Ei dans le repère de l'effecteur
        PEi_E = np.array([
            [Re * np.cos(ang2[i])],
            [Re * np.sin(ang2[i])],
            [1]
        ])
        PEi_0 = THEff @ PEi_E

        # Transformation homogène de Ri par rapport à R0
        Rot = np.array([
            [np.cos(ang1[i]), -np.sin(ang1[i])],
            [np.sin(ang1[i]),  np.cos(ang1[i])]
        ])
        trans = np.array([[Rb * np.cos(ang2[i])], [Rb * np.sin(ang2[i])]])
        THRi_0 = np.vstack((
            np.hstack((Rot, trans)),
            np.array([[0, 0, 1]])
        ))

        # Calcul du point Bi (extrémité du bras i)
        PBi_local = np.array([
            [L1 * np.cos(alpha[i]) + L2 * np.cos(alpha[i] + beta[i])],
            [L1 * np.sin(alpha[i]) + L2 * np.sin(alpha[i] + beta[i])],
            [1]
        ])
        PBi = THRi_0 @ PBi_local

        # Contraintes : Bi doit coïncider avec Ei
        F.append(PBi[0, 0] - PEi_0[0, 0])  # x
        F.append(PBi[1, 0] - PEi_0[1, 0])  # y

    return F
