import math
import numpy as np
import pygame
from config import L1, L2, Rb, Re, SINGULARITY_THRESHOLD
from fonctions_mathématiques import (
    rotation_matrix,
    homogene_transform,
    point_with_TH,
    point_with_inv_TH,
    point_in_polygon,
    build_matrix_A,
    build_matrix_B,
    det,
)
from scipy.optimize import root

# ------------------------------------------------------------------
# Limite mécanique des servomoteurs : ±150°  
# ------------------------------------------------------------------
MAX_SERVO_ANGLE = math.radians(150)

# ------------------------------------------------------------------
# Outil graphique : (x, y) m → pixels
# ------------------------------------------------------------------
def to_screen(pos, scale: int = 1000, offset=(400, 300)):
    return int(pos[0] * scale + offset[0]), int(-pos[1] * scale + offset[1])

# ------------------------------------------------------------------
# Classe Robot 3-RRR
# ------------------------------------------------------------------
class Robot3RRR:
    """Robot parallèle plan 3-RRR : cinématique + rendu Pygame."""

    def __init__(self, L1, L2, Rb, Re, ang1, ang2):
        self.L1, self.L2, self.Rb, self.Re = L1, L2, Rb, Re
        self.ang1, self.ang2 = ang1, ang2
        self.pose = (0.0, 0.0, 0.0)
        self.q = None
        self.mode = "analytic"       # analytic | numeric
        self.restricted_mode = False # évite collisions Bi∈effecteur

        # Diagnostics
        self.error_msg = ""
        self.bloque = False
        self.last_detA = None
        self.last_detB = None

    # --------------------------------------------------------------
    # MGI numérique (Newton-Raphson)
    # --------------------------------------------------------------
    def _eq_constraints(self, q, eff):
        al = [q[0], q[2], q[4]]
        be = [q[1], q[3], q[5]]
        R_E = rotation_matrix(eff[2])
        H_E = homogene_transform(R_E, np.array([[eff[0]], [eff[1]]]))
        F = []
        for i in range(3):
            PEi_E = np.array([self.Re*math.cos(self.ang2[i]),
                              self.Re*math.sin(self.ang2[i]), 1])
            PEi_0 = point_with_TH(H_E, PEi_E)
            R_i = rotation_matrix(self.ang1[i])
            Oi = np.array([[self.Rb*math.cos(self.ang2[i])],
                           [self.Rb*math.sin(self.ang2[i])]])
            H_Ri = homogene_transform(R_i, Oi)
            x_bi = self.L1*math.cos(al[i]) + self.L2*math.cos(al[i]+be[i])
            y_bi = self.L1*math.sin(al[i]) + self.L2*math.sin(al[i]+be[i])
            PBi_0 = point_with_TH(H_Ri, np.array([x_bi, y_bi, 1]))
            F.extend([PBi_0[0]-PEi_0[0], PBi_0[1]-PEi_0[1]])
        return np.array(F)

    def inverse_kinematics_numeric(self, pose, q0=None, max_iter=50, tol=1e-6):
        if q0 is None:
            q0 = np.zeros(6)
        res = root(lambda q: self._eq_constraints(q, pose), q0,
                   method="hybr", options={"maxfev": max_iter, "xtol": tol})
        if not res.success:
            raise ValueError("MGI numérique échoué : " + res.message)
        q = res.x
        return [(q[0], q[1]), (q[2], q[3]), (q[4], q[5])]

    # --------------------------------------------------------------
    # MGI analytique + singularités + limite servo
    # --------------------------------------------------------------
    def compute_kinematics(self, eff):
        x, y, theta = eff
        H_E = homogene_transform(rotation_matrix(theta), np.array([[x], [y]]))
        Ce = np.array([x, y])

        gam, ds, es, q_list = [], [], [], []
        cross = lambda a, b: a[0]*b[1] - a[1]*b[0]

        for i in range(3):
            H_Ri = homogene_transform(
                rotation_matrix(self.ang1[i]),
                np.array([[self.Rb*math.cos(self.ang2[i])],
                          [self.Rb*math.sin(self.ang2[i])]]))

            PEi_0 = H_E @ np.array([[self.Re*math.cos(self.ang2[i])],
                                    [self.Re*math.sin(self.ang2[i])],
                                    [1]])
            PEi_i = point_with_inv_TH(H_Ri, PEi_0)
            xi, yi = PEi_i[0, 0], PEi_i[1, 0]

            cosb = (xi**2 + yi**2 - self.L1**2 - self.L2**2) / (2*self.L1*self.L2)
            if abs(cosb) > 1:
                raise ValueError("Pose hors espace de travail")
            beta = math.acos(cosb)
            alpha = math.atan2(yi, xi) - math.atan2(
                self.L2*math.sin(beta), self.L1+self.L2*math.cos(beta))

            # Limite Dynamixel ±45°
            if not (-MAX_SERVO_ANGLE <= alpha <= MAX_SERVO_ANGLE):
                raise ValueError(f"Limite servo dépassée : α{i+1}={math.degrees(alpha):.1f}°")

            Oi = (self.Rb*math.cos(self.ang2[i]), self.Rb*math.sin(self.ang2[i]))
            Bi = (Oi[0] + self.L1*math.cos(alpha+self.ang1[i]),
                  Oi[1] + self.L1*math.sin(alpha+self.ang1[i]))
            Ei = (Bi[0] + self.L2*math.cos(alpha+beta+self.ang1[i]),
                  Bi[1] + self.L2*math.sin(alpha+beta+self.ang1[i]))

            ui = np.array([Ei[0]-Bi[0], Ei[1]-Bi[1]])
            if np.linalg.norm(ui) == 0:
                raise ValueError("Vecteur nul – configuration dégénérée")
            ui_n = ui/np.linalg.norm(ui)

            gam.append(math.atan2(ui_n[1], ui_n[0]))
            ds.append(cross(Ce - np.array(Bi), ui_n))
            es.append(cross(np.array(Oi) - np.array(Bi), ui_n))
            q_list.append((alpha, beta, Bi, Ei))

        A = build_matrix_A(gam, ds)
        B = build_matrix_B(es)
        self.last_detA, self.last_detB = detA, detB = det(A), det(B)

        if abs(detA) < SINGULARITY_THRESHOLD:
            raise ValueError(f"Singularité parallèle : det(A) ≈ {detA:.2e}")
        if abs(detB) < SINGULARITY_THRESHOLD:
            raise ValueError(f"Singularité série : det(B) ≈ {detB:.2e}")

        if self.restricted_mode:
            poly = [item[3] for item in q_list]
            for (_, _, Bi, _) in q_list:
                if point_in_polygon(Bi, poly):
                    return None
        return q_list

    # --------------------------------------------------------------
    # Interface haut-niveau
    # --------------------------------------------------------------
    def set_pose(self, pose):
        try:
            if self.mode == "analytic":
                q = self.compute_kinematics(pose)
                if self.restricted_mode and q is None:
                    return
            else:
                q = self.inverse_kinematics_numeric(pose)
            self.pose, self.q = pose, q
            self.error_msg, self.bloque = "", False
        except Exception as e:
            self.error_msg, self.bloque, self.q = str(e), True, None

    # --------------------------------------------------------------
    # Dessin Pygame
    # --------------------------------------------------------------
    def draw(self, surf):
        if self.q is None:
            return
        tri = []
        for i, (_, _, Bi, Ei) in enumerate(self.q):
            Oi = (self.Rb*math.cos(self.ang2[i]), self.Rb*math.sin(self.ang2[i]))
            col = (200,0,0) if self.bloque else (0,0,200)
            pygame.draw.line(surf, col, to_screen(Oi), to_screen(Bi), 3)
            pygame.draw.line(surf, (0,180,0), to_screen(Bi), to_screen(Ei), 3)
            pygame.draw.circle(surf, (255,140,0), to_screen(Bi), 5)
            pygame.draw.circle(surf, (220,0,0), to_screen(Ei), 5)
            tri.append(to_screen(Ei))
        pygame.draw.polygon(surf, (0,0,0), tri, 2)

        if self.bloque and self.error_msg:
            font_big = pygame.font.SysFont("Arial", 22)
            txt = font_big.render("Erreur : "+self.error_msg, True, (200,0,0))
            surf.blit(txt, txt.get_rect(center=(400, 50)))

    def draw_info(self, surf, font, width, drawing_mode):
        x, y, th = self.pose
        surf.blit(font.render(f"x={x:.2f} m, y={y:.2f} m, θ={math.degrees(th):.1f}°",
                              True, (0,0,0)), (10,10))
        surf.blit(font.render(f"Mode MGI : {self.mode.upper()}", True, (0,0,0)), (10,30))
        surf.blit(font.render(f"det(A)={self.last_detA:+.2e}", True, (0,0,0)), (10,50))
        surf.blit(font.render(f"det(B)={self.last_detB:+.2e}", True, (0,0,0)), (10,70))
        surf.blit(font.render(f"Restreint : {'ON' if self.restricted_mode else 'OFF'}",
                              True, (0,0,0)), (width-230,150))
        surf.blit(font.render(f"Dessin traj : {'ON' if drawing_mode else 'OFF'}",
                              True, (0,0,0)), (width-230,170))
        if self.error_msg and self.bloque:
            surf.blit(font.render("Erreur : "+self.error_msg, True, (200,0,0)), (10,90))
