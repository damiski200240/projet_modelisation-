# methode pour calucler le modele cinematique inverse d'une maniere qui donne les solutions rapidement avec le coude bas et haut. Ceci sera couplé avec jacobien_cond pour trouver les singularités 
# et de enfin trouver les params qui maximise le workspace
# 

from numpy import sqrt, sin , cos, pi, arctan
import numpy as np
def ikm(param, x, y, theta_e):
    K, l1, l2, R = param
    i = np.arange(1, 4)
    xA = K * np.cos(2 * i * np.pi / 3 + np.pi / 2)
    yA = K * np.sin(2 * i * np.pi / 3 + np.pi / 2)
    xCP = R * np.cos(theta_e + 2 * i * np.pi / 3 - np.pi / 2)
    yCP = R * np.sin(theta_e + 2 * i * np.pi / 3 - np.pi / 2)
    a = (K ** 2 + R ** 2 + l1 ** 2 - l2 ** 2 + x ** 2 + y ** 2 +
         2 * xA * xCP - 2 * xA * x - 2 * xCP * x +
         2 * yA * yCP - 2 * yA * y - 2 * yCP * y)
    b = 2 * l1 * (xA + xCP - x)
    c = 2 * l1 * (yA + yCP - y)
    sqrt_arg = c ** 2 + b ** 2 - a ** 2
    thp = np.full(3, np.nan)
    thm = np.full(3, np.nan)
    for idx in range(3):
        if sqrt_arg[idx] >= 0:
            sqrt_val = np.sqrt(sqrt_arg[idx])
            thp[idx] = 2 * np.arctan((-c[idx] + sqrt_val) / (a[idx] - b[idx]))
            thm[idx] = 2 * np.arctan((-c[idx] - sqrt_val) / (a[idx] - b[idx]))
    th1 = np.array([thp[0], thm[0]])
    th2 = np.array([thp[1], thm[1]])
    th3 = np.array([thp[2], thm[2]])
    return th1, th2, th3
