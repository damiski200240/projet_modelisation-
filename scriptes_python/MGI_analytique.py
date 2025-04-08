import numpy as np

# Variables globales à définir ailleurs
L1 = None 
L2 = None 
Rb = None 
Re = None 

def MGI_analytique(eff):
    global L1, L2, Rb, Re

    # Matrice de rotation + translation de l'effecteur
    theta = eff[2]
    RotEff = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    Transl = np.array([[eff[0]], [eff[1]]])
    THEff = np.block([
        [RotEff, Transl],
        [np.zeros((1, 2)), [1]]
    ])

    ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]
    ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

    q = []

    for i in range(3):
        # Transformation homogène du repère R_i
        Rot = np.array([
            [np.cos(ang1[i]), -np.sin(ang1[i])],
            [np.sin(ang1[i]),  np.cos(ang1[i])]
        ])
        TH = np.block([
            [Rot, np.array([[Rb * np.cos(ang2[i])], [Rb * np.sin(ang2[i])]])],
            [np.zeros((1, 2)), [1]]
        ])

        # Point Ei dans le repère effecteur
        PEi_E = np.array([
            [Re * np.cos(ang2[i])],
            [Re * np.sin(ang2[i])],
            [1]
        ])

        # Position de Ei dans le repère monde
        PEi_0 = THEff @ PEi_E

        # Position de Ei dans le repère R_i
        PEi_i = np.linalg.inv(TH) @ PEi_0
        x, y = PEi_i[0, 0], PEi_i[1, 0]

        # Cinématique inverse 2R
        aux = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(aux) < 1:
            beta = np.arccos(aux)  # solution coude en haut
        else:
            beta = 0
            print("⚠️ Problème d'atteignabilité")

        alpha = np.arctan2(y, x) - np.arctan2(L2 * np.sin(beta), L1 + L2 * np.cos(beta))
        q.append([alpha, beta])

    return np.array(q)
