import numpy as np
import matplotlib.pyplot as plt

# Déclaration des longueurs globales
global L1, L2, Rb, Re

L1=0.10  #long segment 1
L2=0.10  #long segment 2
Rb=0.1322594  # Rayon base
Re=0.07 # Rayon effecteur
 

def trace_rob(q):
    global L1, L2, Rb, Re

    alpha1, beta1 = q[0], q[1]
    alpha2, beta2 = q[2], q[3]
    alpha3, beta3 = q[4], q[5]

    # Rotations de base
    angle1 = 2 * np.pi / 3
    angle2 = 4 * np.pi / 3

    Rot1 = np.array([
        [np.cos(angle1), -np.sin(angle1)],
        [np.sin(angle1),  np.cos(angle1)]
    ])

    Rot2 = np.array([
        [np.cos(angle2), -np.sin(angle2)],
        [np.sin(angle2),  np.cos(angle2)]
    ])

    # Bras 1
    P10 = np.array([0, -Rb])
    P11 = np.array([L1 * np.cos(alpha1), L1 * np.sin(alpha1)]) + P10
    P12 = np.array([
        L1 * np.cos(alpha1) + L2 * np.cos(alpha1 + beta1),
        L1 * np.sin(alpha1) + L2 * np.sin(alpha1 + beta1)
    ]) + P10

    # Bras 2
    base2 = np.array([Rb * np.sqrt(3)/2, Rb/2])
    vec2_1 = np.array([L1 * np.cos(alpha2), L1 * np.sin(alpha2)])
    vec2_2 = np.array([
        L1 * np.cos(alpha2) + L2 * np.cos(alpha2 + beta2),
        L1 * np.sin(alpha2) + L2 * np.sin(alpha2 + beta2)
    ])
    P20 = base2
    P21 = Rot1 @ vec2_1 + base2
    P22 = Rot1 @ vec2_2 + base2

    # Bras 3
    base3 = np.array([-Rb * np.sqrt(3)/2, Rb/2])
    vec3_1 = np.array([L1 * np.cos(alpha3), L1 * np.sin(alpha3)])
    vec3_2 = np.array([
        L1 * np.cos(alpha3) + L2 * np.cos(alpha3 + beta3),
        L1 * np.sin(alpha3) + L2 * np.sin(alpha3 + beta3)
    ])
    P30 = base3
    P31 = Rot2 @ vec3_1 + base3
    P32 = Rot2 @ vec3_2 + base3

    # Tracé
    plt.figure()
    plt.plot([P10[0], P11[0], P12[0]], [P10[1], P11[1], P12[1]], label='Bras 1')
    plt.plot([P20[0], P21[0], P22[0]], [P20[1], P21[1], P22[1]], label='Bras 2')
    plt.plot([P30[0], P31[0], P32[0]], [P30[1], P31[1], P32[1]], label='Bras 3')

    # Contour de l'effecteur (triangle)
    plt.plot(
        [P12[0], P22[0], P32[0], P12[0]],
        [P12[1], P22[1], P32[1], P12[1]],
        'k-', linewidth=2, label='Effecteur'
    )

    plt.axis('equal')
    plt.grid(True)
    plt.title("Robot parallèle 3R")
    plt.legend()
    plt.show()
