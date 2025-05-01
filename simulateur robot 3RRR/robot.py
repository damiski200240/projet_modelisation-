import math
import numpy as np
import pygame
from config import L1, L2, Rb, Re, SINGULARITY_THRESHOLD
from fonctions_mathématiques import (
    rotation_matrix,
    homogene_transform,
    point_with_TH,
    point_with_inv_TH,
    safe_acos,
    point_in_polygon
)
from scipy.optimize import root

def to_screen(pos):
    scale = 1000
    offset = (400, 300)
    return int(pos[0] * scale + offset[0]), int(-pos[1] * scale + offset[1])

class Robot3RRR:
    def __init__(self, L1, L2, Rb, Re, ang1, ang2):
        self.L1 = L1
        self.L2 = L2
        self.Rb = Rb
        self.Re = Re
        self.ang1 = ang1
        self.ang2 = ang2
        self.pose = (0, 0, 0)
        self.q = None
        self.error_msg = ""
        self.restricted_mode = False
        self.mode = "analytic"
        self.bloque = False

    def solve_eq_NL(self, q, eff):
        alpha = [q[0], q[2], q[4]]
        beta = [q[1], q[3], q[5]]
        R_E = rotation_matrix(eff[2])
        T_E = np.array([[eff[0]], [eff[1]]])
        H_E = homogene_transform(R_E, T_E)
        F = []
        for i in range(3):
            PEi_E = np.array([self.Re * math.cos(self.ang2[i]), self.Re * math.sin(self.ang2[i]), 1])
            PEi_0 = point_with_TH(H_E, PEi_E)
            R_i = rotation_matrix(self.ang1[i])
            Oi = np.array([[self.Rb * math.cos(self.ang2[i])], [self.Rb * math.sin(self.ang2[i])]])
            H_Ri = homogene_transform(R_i, Oi)

            x_bi = self.L1 * math.cos(alpha[i]) + self.L2 * math.cos(alpha[i] + beta[i])
            y_bi = self.L1 * math.sin(alpha[i]) + self.L2 * math.sin(alpha[i] + beta[i])
            PBi_local = np.array([x_bi, y_bi, 1])
            PBi_0 = point_with_TH(H_Ri, PBi_local)

            F.append(PBi_0[0] - PEi_0[0])
            F.append(PBi_0[1] - PEi_0[1])
        return np.array(F)

    def inverse_kinematics_numeric(self, pose, q0=None, max_iter=50, tol=1e-6):
        if q0 is None:
            q0 = np.zeros(6)
        result = root(lambda q: self.solve_eq_NL(q, pose), q0, method='hybr', options={'maxfev': max_iter, 'xtol': tol})
        if result.success:
            q = result.x
            return [(q[0], q[1]), (q[2], q[3]), (q[4], q[5])]
        else:
            raise ValueError("MGI numérique échoué")

    def compute_kinematics(self, eff):
        q = []
        x, y, theta = eff
        RotEff = rotation_matrix(theta)
        Transl = np.array([[x], [y]])
        THEff = homogene_transform(RotEff, Transl)

        for i in range(3):
            Rot = rotation_matrix(self.ang1[i])
            pos_Oi = np.array([[self.Rb * math.cos(self.ang2[i])],
                               [self.Rb * math.sin(self.ang2[i])]])
            TH = homogene_transform(Rot, pos_Oi)

            PEi_E = np.array([[self.Re * math.cos(self.ang2[i])],
                              [self.Re * math.sin(self.ang2[i])],
                              [1]])
            PEi_0 = THEff @ PEi_E
            PEi_i = point_with_inv_TH(TH, PEi_0)

            x_i, y_i = PEi_i[0, 0], PEi_i[1, 0]
            aux = (x_i ** 2 + y_i ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
            if abs(aux) > 1:
                raise ValueError("Singularité série (hors de portée)")

            beta = math.acos(aux)
            alpha = math.atan2(y_i, x_i) - math.atan2(self.L2 * math.sin(beta),
                                                      self.L1 + self.L2 * math.cos(beta))

            Oi = (self.Rb * math.cos(self.ang2[i]), self.Rb * math.sin(self.ang2[i]))
            Bi = (Oi[0] + self.L1 * math.cos(alpha + self.ang1[i]),
                  Oi[1] + self.L1 * math.sin(alpha + self.ang1[i]))
            Ei = (Bi[0] + self.L2 * math.cos(alpha + beta + self.ang1[i]),
                  Bi[1] + self.L2 * math.sin(alpha + beta + self.ang1[i]))

            v1 = np.array([Ei[0] - Oi[0], Ei[1] - Oi[1]])
            v2 = np.array([Bi[0] - Oi[0], Bi[1] - Oi[1]])
            if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
                raise ValueError("Vecteur nul")
            ang = math.acos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))
            if ang > math.radians(100):
                raise ValueError("Singularité parallèle (bras mal orienté)")

            q.append((alpha, beta, Bi, Ei))

        # Mode restreint : on ignore simplement la pose si Bi sous l'effecteur (aucune erreur ni blocage)
        if self.restricted_mode:
            effecteur_poly = [item[3] for item in q]
            for (_, _, Bi, _) in q:
                if point_in_polygon(Bi, effecteur_poly):
                    return None  # pose ignorée sans message ni blocage

        return q

    def set_pose(self, new_pose):
        try:
            if self.mode == "analytic":
                q = self.compute_kinematics(new_pose)
                if self.restricted_mode and q is None:
                    return  # pose interdite en mode restreint, on ignore
            elif self.mode == "numeric":
                q = self.inverse_kinematics_numeric(new_pose)
            else:
                raise ValueError("Méthode inconnue")

            self.pose = new_pose
            self.q = q
            self.error_msg = ""
            self.bloque = False

        except Exception as e:
            self.q = None
            self.bloque = True
            self.error_msg = str(e)

    def draw(self, surface):
        if self.q is None:
            return
        triangle_points = []
        for i in range(3):
            alpha, beta, Bi, Ei = self.q[i]
            Oi = (self.Rb * math.cos(self.ang2[i]), self.Rb * math.sin(self.ang2[i]))
            color_base = (255, 0, 0) if self.bloque else (0, 0, 255)

            pygame.draw.line(surface, color_base, to_screen(Oi), to_screen(Bi), 3)
            pygame.draw.circle(surface, (255, 150, 0), to_screen(Bi), 5)
            pygame.draw.line(surface, (0, 200, 0), to_screen(Bi), to_screen(Ei), 3)
            pygame.draw.circle(surface, (255, 0, 0), to_screen(Ei), 5)
            triangle_points.append(to_screen(Ei))

        pygame.draw.polygon(surface, (0, 0, 0), triangle_points, 2)

        if self.bloque and self.error_msg:
            font_big = pygame.font.SysFont("Arial", 22)
            err_text = font_big.render(f"Erreur: {self.error_msg}", True, (200, 0, 0))
            rect = err_text.get_rect(center=(400, 50))
            surface.blit(err_text, rect)

    def draw_info(self, surface, font, width, drawing_mode):
        surface.blit(font.render(f"Pose: x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, θ={math.degrees(self.pose[2]):.1f}°", True, (0, 0, 0)), (10, 10))
        surface.blit(font.render(f"Mode MGI: {self.mode.upper()}", True, (0, 0, 0)), (10, 30))
        surface.blit(font.render(f"Mode restreint: {'ON' if self.restricted_mode else 'OFF'}", True, (0, 0, 0)), (width - 220, 150))
        surface.blit(font.render(f"Mode dessin: {'ON' if drawing_mode else 'OFF'}", True, (0, 0, 0)), (width - 220, 170))

        if self.error_msg and self.bloque:
            surface.blit(font.render("Erreur: " + self.error_msg, True, (200, 0, 0)), (10, 50))
