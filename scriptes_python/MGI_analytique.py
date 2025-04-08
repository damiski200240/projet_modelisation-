import numpy as np

# Variables globales
global L1, L2, Rb, Re

L1=0.10  #long segment 1
L2=0.10  #long segment 2
Rb=0.1322594  # Rayon base
Re=0.07 # Rayon effecteur

def MGI_analytique(eff):
    global L1, L2, Rb, Re

    theta = eff[2]
    RotEff = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    Transl = np.array([[eff[0]], [eff[1]]])  # colonne 2x1
    bottom_row = np.array([[0, 0, 1]])
    THEff = np.vstack((
        np.hstack((RotEff, Transl)),  # 2x3
        bottom_row                   # 1x3
    ))

    ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]
    ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

    q = []

    for i in range(3):
        Rot = np.array([
            [np.cos(ang1[i]), -np.sin(ang1[i])],
            [np.sin(ang1[i]),  np.cos(ang1[i])]
        ])
        trans = np.array([[Rb * np.cos(ang2[i])], [Rb * np.sin(ang2[i])]])
        TH = np.vstack((
            np.hstack((Rot, trans)),
            bottom_row
        ))

        PEi_E = np.array([
            [Re * np.cos(ang2[i])],
            [Re * np.sin(ang2[i])],
            [1]
        ])

        PEi_0 = THEff @ PEi_E
        PEi_i = np.linalg.inv(TH) @ PEi_0
        x, y = PEi_i[0, 0], PEi_i[1, 0]

        aux = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(aux) < 1:
            beta = np.arccos(aux)
        else:
            beta = 0
            print(f"⚠️ Problème d'atteignabilité pour le bras {i+1}")

        alpha = np.arctan2(y, x) - np.arctan2(L2 * np.sin(beta), L1 + L2 * np.cos(beta))
        q.append([alpha, beta])

    return np.array(q)
