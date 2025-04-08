import numpy as np

 
L1 = None
L2 = None
Rb = None
Re = None

def solve_eq_NL(q, eff):
    global L1, L2, Rb, Re  # déclaration des variables globales utilisées dans la fonction

    # Extraire alpha et beta du vecteur q
    alpha = [q[0], q[2], q[4]]
    beta  = [q[1], q[3], q[5]]

    # Angles de base
    ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]
    ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

    # Matrice de rotation et transformation homogène de l'effecteur
    RotEff = np.array([
        [np.cos(eff[2]), -np.sin(eff[2])],
        [np.sin(eff[2]),  np.cos(eff[2])]
    ])
    Transl = np.array([[eff[0]], [eff[1]]])
    THEff = np.block([
        [RotEff, Transl],
        [np.zeros((1, 2)), [1]]
    ])

    F = []

    for i in range(3):
        # Point Ei dans le repère effecteur
        PEi_E = np.array([
            [Re * np.cos(ang2[i])],
            [Re * np.sin(ang2[i])],
            [1]
        ])
        # Transformation dans le repère monde
        PEi_0 = THEff @ PEi_E

        # Transformation homogène de Ri par rapport à R0
        Rot = np.array([
            [np.cos(ang1[i]), -np.sin(ang1[i])],
            [np.sin(ang1[i]),  np.cos(ang1[i])]
        ])
        THRi_0 = np.block([
            [Rot, np.array([[Rb * np.cos(ang2[i])], [Rb * np.sin(ang2[i])]])],
            [np.zeros((1, 2)), [1]]
        ])

        # Calcul du point Bi (extrémité du bras)
        PBi_local = np.array([
            [L1 * np.cos(alpha[i]) + L2 * np.cos(alpha[i] + beta[i])],
            [L1 * np.sin(alpha[i]) + L2 * np.sin(alpha[i] + beta[i])],
            [1]
        ])
        PBi = THRi_0 @ PBi_local

        # Contraintes d'égalité Bi == Ei
        F.append(PBi[0, 0] - PEi_0[0, 0])
        F.append(PBi[1, 0] - PEi_0[1, 0])

    return F
