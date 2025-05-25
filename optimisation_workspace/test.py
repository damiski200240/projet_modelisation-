import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.ops import unary_union

def ikm(param, x, y, alpha):
    K, l1, l2, R = param
    i = np.arange(1, 4)
    xA = K * np.cos(2 * i * np.pi / 3 + np.pi / 2)
    yA = K * np.sin(2 * i * np.pi / 3 + np.pi / 2)
    xCP = R * np.cos(alpha + 2 * i * np.pi / 3 - np.pi / 2)
    yCP = R * np.sin(alpha + 2 * i * np.pi / 3 - np.pi / 2)
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

def ikm_phi_psi(param, theta, x, y, alpha):
    K, l1, l2, R = param
    th1, th2, th3 = theta[:, 0], theta[:, 1], theta[:, 2]
    xA = K * np.cos(np.pi / 6)
    yA = K * np.sin(np.pi / 6)
    cph1 = (xA - l1 * np.cos(th1) - R * np.cos(alpha + np.pi / 6) + x) / l2
    sph1 = (yA - l1 * np.sin(th1) - R * np.sin(alpha + np.pi / 6) + y) / l2
    cph2 = (-xA - l1 * np.cos(th2) - R * np.cos(alpha + 5 * np.pi / 6) + x) / l2
    sph2 = (yA - l1 * np.sin(th2) - R * np.sin(alpha + 5 * np.pi / 6) + y) / l2
    cph3 = (-(l1 * np.cos(th3) + R * np.cos(alpha - np.pi / 2) - x)) / l2
    sph3 = (-(K + l1 * np.sin(th3) + R * np.sin(alpha - np.pi / 2) - y)) / l2
    ph1 = np.arctan2(sph1, cph1)
    ph2 = np.arctan2(sph2, cph2)
    ph3 = np.arctan2(sph3, cph3)
    psi1 = np.pi - ph1 + np.pi / 6 + alpha
    psi2 = np.pi - ph2 + 5 * np.pi / 6 + alpha
    psi3 = np.pi - ph3 + 3 * np.pi / 2 + alpha
    return ph1, ph2, ph3, psi1, psi2, psi3

def det_jacobian(param, theta, phi, alpha):
    K, L1, L2, R = param
    th1, th2, th3 = theta
    ph1, ph2, ph3 = phi
    dJth = L1 ** 3 * np.sin(ph1 - th1) * np.sin(ph2 - th2) * np.sin(ph3 - th3)
    Jx = np.array([
        [-np.cos(ph1), -np.sin(ph1), R * np.sin(ph1 - alpha - np.pi / 6)],
        [-np.cos(ph2), -np.sin(ph2), R * np.sin(ph2 - alpha - 5 * np.pi / 6)],
        [-np.cos(ph3), -np.sin(ph3), R * np.sin(ph3 - alpha - 3 * np.pi / 2)]
    ])
    dJx = np.linalg.det(Jx)
    return dJth, dJx

def combine_det(param, th1, th2, th3, x, y, a):
    from itertools import product
    th_combos = np.array(list(product(th1, th2, th3)))
    detJx = np.zeros(8)
    detJth = np.zeros(8)
    for k, th in enumerate(th_combos):
        if np.any(np.isnan(th)):
            detJth[k] = np.nan
            detJx[k] = np.nan
            continue
        ph1, ph2, ph3, *_ = ikm_phi_psi(param, np.array([th]), x, y, a)
        ph = [ph1[0], ph2[0], ph3[0]]
        if np.any(np.isnan(ph)):
            detJth[k] = np.nan
            detJx[k] = np.nan
            continue
        Jth, Jx = det_jacobian(param, th, ph, a)
        if np.isnan(Jth) or np.isnan(Jx):
            detJth[k] = np.nan
            detJx[k] = np.nan
        else:
            detJth[k] = Jth
            detJx[k] = Jx
            
    return detJth, detJx
def get_coord(param, theta, phi, alpha, rot):
    """
    Computes the position of the end effector given joint angles and parameters.
    """
    K, l1, l2, R = param

    # Rotation matrix
    rot_mat = np.array([
        [np.cos(rot), -np.sin(rot)],
        [np.sin(rot),  np.cos(rot)]
    ])

    # Vector coordinates
    OA = rot_mat @ np.array([0, K])
    AB = np.array([l1 * np.cos(theta), l1 * np.sin(theta)])
    BC = np.array([l2 * np.cos(phi), l2 * np.sin(phi)])
    CP = rot_mat @ np.array([R * np.cos(alpha - np.pi / 2), R * np.sin(alpha - np.pi / 2)])

    # End-effector's final position
    end_effector = OA + AB + BC + CP

    return end_effector


def angdiff(alpha, beta):
    """
    Calculates the difference between angles alpha and beta,
    wrapped to the interval [-pi, pi].
    """
    diff = alpha - beta
    return (diff + np.pi) % (2 * np.pi) - np.pi

# Main plotting and computation (singularitiesloci.m equivalent)
def main():
    nb_points = 100
    alpha = 0
    K = 150
    l1 = 98.18861278
    l2 = 98.18861278
    R = 77.89302838
    param = [K, l1, l2, R]
    limit = 300 * np.pi / 180
    mode = '+ + +'
    home_pos = [0, 0, 0]
    orientation = 0 * np.pi / 180
    valid_points = 0
    # Workspace calculation omitted for brevity

    coord = np.linspace(-210, 210, nb_points)
    detJth = np.zeros((8, nb_points, nb_points))
    detJx = np.zeros((8, nb_points, nb_points))

    for j in range(nb_points):
        for k in range(nb_points):
            th1, th2, th3 = ikm(param, coord[j], coord[k], alpha)
            if (np.isreal(th1).all() and np.isreal(th2).all() and np.isreal(th3).all()):
                dJth, dJx = combine_det(param, th1, th2, th3, coord[j], coord[k], alpha)
                detJth[:, k, j] = dJth
                detJx[:, k, j] = dJx
                if not np.all(np.isnan(dJx)):
                    valid_points += 1
    print(f"Number of valid points: {valid_points}")

    Mx = np.nanmax(np.abs(detJx))

    detJx /= Mx
    Mth = np.nanmax(np.abs(detJth))
    detJth /= Mth

    plt.figure()
    plt.imshow(np.abs(detJx[0]), extent=[-210, 210, -210, 210], origin='lower')
    plt.colorbar(label='Absolute value of det(Jx) (normalized)')
    plt.title('Direct (type II) singularity contours')
    plt.xlabel('x (mm)')
    plt.ylabel('y (mm)')
    plt.axis('equal')
    plt.show()

    plt.figure()
    plt.imshow(np.abs(detJth[0]), extent=[-210, 210, -210, 210], origin='lower')
    plt.colorbar(label='Absolute value of det(Jth) (normalized)')
    plt.title('Direct (type I) singularity contours')
    plt.xlabel('x (mm)')
    plt.ylabel('y (mm)')
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    main()
