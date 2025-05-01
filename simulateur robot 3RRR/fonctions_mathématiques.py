import math
import numpy as np

# --------------------------
#  Fonctions mathématiques
# --------------------------

def deg2rad(deg):
    """Convertit degrés → radians"""
    return deg * math.pi / 180

def rad2deg(rad):
    """Convertit radians → degrés"""
    return rad * 180 / math.pi

def cos(x):
    return np.cos(x)

def sin(x):
    return np.sin(x)



def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def rotation_matrix(theta):
    """Retourne la matrice de rotation 2D pour un angle theta"""
    return np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta),  math.cos(theta)]
    ])

def rotate_point(p, theta, center=(0, 0)):
    """Fait tourner le point p autour de center d'un angle theta"""
    R = rotation_matrix(theta)
    p_rel = np.array(p) - np.array(center)
    p_rot = R @ p_rel + np.array(center)
    return p_rot

def homogene_transform(R, T):
    """Construit une matrice homogène 3x3 à partir de R (2x2) et T (2x1)"""
    H = np.eye(3)
    H[:2, :2] = R
    H[:2, 2] = T.flatten()
    return H

def point_with_TH(H, point):
    """Transforme un point (x, y, 1) du repère local au global avec la matrice homogène H"""
    return H @ np.array(point)

def point_with_inv_TH(H, point):
    """Transforme un point global en repère local en inversant H"""
    return np.linalg.inv(H) @ np.array(point)



def distance(p1, p2):
    """Distance euclidienne entre deux points 2D"""
    return np.linalg.norm(np.array(p1) - np.array(p2))

def vector_angle(v1, v2):
    """Angle entre deux vecteurs"""
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    dot = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
    return math.acos(dot)

def safe_acos(x):
    """Renvoie acos(x) tout en gérant les erreurs numériques (|x| > 1)"""
    if x > 1: x = 1
    if x < -1: x = -1
    return math.acos(x)



# --------------------------
# Fonctions matrices du modèle cinématique
# --------------------------

def det(matrix):
    """Calcul du déterminant d'une matrice carrée"""
    return np.linalg.det(matrix)

def is_near_zero(value, threshold=1e-4):
    """Test si une valeur est proche de zéro (singularité)"""
    return abs(value) < threshold
 


def build_matrix_A(gammas, ds):
    """
    Construit la matrice A à partir des angles gamma_i et bras de levier d_i
    gammas : liste de 3 angles gamma_i [rad]
    ds     : liste de 3 distances d_i
    """
    A = np.zeros((3, 3))
    for i in range(3):
        A[i, 0] = math.cos(gammas[i])
        A[i, 1] = math.sin(gammas[i])
        A[i, 2] = ds[i]
    return A

def build_matrix_B(es):
    """
    Construit la matrice B diagonale à partir des bras de levier e_i
    es : liste de 3 distances e_i
    """
    return np.diag(es)


 
