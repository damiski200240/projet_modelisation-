import pygame
import math
import numpy as np

# -------- Configuration Pygame --------
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Robot 3RRR - Contrôle clavier avec Mode Dessin")
clock = pygame.time.Clock()
center = (width // 2, height // 2)
font = pygame.font.SysFont("Arial", 18)

# -------- Boutons Reset & Terminé --------
reset_button_rect = pygame.Rect(width - 110, 10, 100, 30)
termine_button_rect = pygame.Rect(width - 110, 50, 100, 30)

# -------- Paramètres du robot --------
L1 = 100
L2 = 100
Rb = 132.26
Re = 70
ang1 = [0, 2 * math.pi / 3, 4 * math.pi / 3]
ang2 = [-math.pi / 2, math.pi / 6, 5 * math.pi / 6]

delta_pos = 5               # déplacement par frame
delta_theta = math.radians(2)  # variation d'angle par frame

# -------- Fonctions utilitaires --------
def rotation(theta):
    return np.array([[math.cos(theta), -math.sin(theta)],
                     [math.sin(theta),  math.cos(theta)]])

def to_screen(p):
    return int(center[0] + p[0]), int(center[1] - p[1])

def draw_reset_button(surface):
    pygame.draw.rect(surface, (180, 180, 180), reset_button_rect)
    text = font.render("RESET", True, (0, 0, 0))
    text_rect = text.get_rect(center=reset_button_rect.center)
    surface.blit(text, text_rect)

def draw_termine_button(surface):
    pygame.draw.rect(surface, (180, 180, 180), termine_button_rect)
    text = font.render("TERMINÉ", True, (0, 0, 0))
    text_rect = text.get_rect(center=termine_button_rect.center)
    surface.blit(text, text_rect)

def draw_instructions(surface):
    instructions = [
        "Touches:",
        "Flèches : déplacement (x, y)",
        "Q / D  : rotation de l'effecteur",
        "R      : Reset position",
        "M      : Mode restreint ON/OFF",
        "Z      : Mode dessin ON/OFF",
        "Terminé: bouton ou (Esc) pour quitter"
    ]
    for i, line in enumerate(instructions):
        text_surf = font.render(line, True, (0, 0, 0))
        surface.blit(text_surf, (10, height - 20 * (len(instructions) - i)))

# def draw_mode_status(surface, restricted, dessin):
    # mode_text = "Restreint: ON" if restricted else "Restreint: OFF"
    # mode_surf = font.render(mode_text, True, (0, 0, 0))
    # surface.blit(mode_surf, (width - 200, 10))
    # dessin_text = "Dessin: ON" if dessin else "Dessin: OFF"
    # dessin_surf = font.render(dessin_text, True, (0, 0, 0))
    # surface.blit(dessin_surf, (width - 200, 30))

def point_in_polygon(point, polygon):
    """Retourne True si le point est à l'intérieur du polygone (algorithme du lancer de rayon)."""
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

# -------- Classe Robot3RRR --------
class Robot3RRR:
    def __init__(self, L1, L2, Rb, Re, ang1, ang2):
        self.L1 = L1
        self.L2 = L2
        self.Rb = Rb
        self.Re = Re
        self.ang1 = ang1
        self.ang2 = ang2
        self.pose = (0, 0, 0)  # Pose courante (x, y, theta)
        self.q = None        # Configuration cinématique stockée
        self.error_msg = ""
        self.restricted_mode = False  # Mode restreint OFF par défaut
    
    def compute_kinematics(self, eff):
        """Calcule la cinématique inverse analytique.
           Renvoie une liste de tuples (alpha, beta, Bi, Ei) pour i=0,1,2.
           Lève une exception si la pose est inatteignable."""
        q = []
        x, y, theta = eff
        RotEff = rotation(theta)
        Transl = np.array([[x], [y]])
        THEff = np.block([[RotEff, Transl], [np.zeros((1,2)), np.ones((1,1))]])
        
        for i in range(3):
            Rot = rotation(self.ang1[i])
            pos_Oi = np.array([[self.Rb * math.cos(self.ang2[i])],
                               [self.Rb * math.sin(self.ang2[i])]])
            TH = np.block([[Rot, pos_Oi], [np.zeros((1,2)), np.ones((1,1))]])
            
            PEi_E = np.array([[self.Re * math.cos(self.ang2[i])],
                              [self.Re * math.sin(self.ang2[i])],
                              [1]])
            PEi_0 = THEff @ PEi_E
            try:
                PEi_i = np.linalg.inv(TH) @ PEi_0
            except np.linalg.LinAlgError:
                raise ValueError("Problème d'inversion de matrice")
            
            x_i, y_i = PEi_i[0, 0], PEi_i[1, 0]
            aux = (x_i ** 2 + y_i ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
            if abs(aux) > 1:
                raise ValueError("Position hors de portée")
            beta = math.acos(aux)
            alpha = math.atan2(y_i, x_i) - math.atan2(self.L2 * math.sin(beta),
                                                        self.L1 + self.L2 * math.cos(beta))
            Oi = (self.Rb * math.cos(self.ang2[i]), self.Rb * math.sin(self.ang2[i]))
            Bi = (Oi[0] + self.L1 * math.cos(alpha + self.ang1[i]),
                  Oi[1] + self.L1 * math.sin(alpha + self.ang1[i]))
            Ei = (Bi[0] + self.L2 * math.cos(alpha + beta + self.ang1[i]),
                  Bi[1] + self.L2 * math.sin(alpha + beta + self.ang1[i]))
            
            # Vérification mécanique : angle entre (Oi->Bi) et (Oi->Ei) < 100°
            v1 = np.array([Ei[0] - Oi[0], Ei[1] - Oi[1]])
            v2 = np.array([Bi[0] - Oi[0], Bi[1] - Oi[1]])
            if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
                raise ValueError("Vecteur nul")
            ang = math.acos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2)), -1, 1))
            if ang > math.radians(100):
                raise ValueError("Bras mal orienté (angle > 100°)")
            q.append((alpha, beta, Bi, Ei))
        
        # Mode restreint : les points d'articulation (Bi) ne peuvent pas être sous le triangle formé par les Ei
        if self.restricted_mode:
            effecteur_poly = [item[3] for item in q]  # Les trois points effecteur
            for (alpha, beta, Bi, Ei) in q:
                if point_in_polygon(Bi, effecteur_poly):
                    raise ValueError("Mode restreint: articulation sous l'effecteur")
                    
        return q
    
    def set_pose(self, new_pose):
        """Met à jour la pose si la nouvelle configuration est atteignable, sinon conserve l'ancienne pose."""
        try:
            q = self.compute_kinematics(new_pose)
            self.pose = new_pose
            self.q = q
            self.error_msg = ""
        except Exception as e:
            self.error_msg = str(e)
            raise e
    
    def draw(self, surface):
        if self.q is None:
            return
        triangle_points = []
        for i in range(3):
            alpha, beta, Bi, Ei = self.q[i]
            Oi = (self.Rb * math.cos(self.ang2[i]), self.Rb * math.sin(self.ang2[i]))
            pygame.draw.line(surface, (0, 0, 255), to_screen(Oi), to_screen(Bi), 3)
            # Affichage du point orange sur l'articulation (Bi)
            pygame.draw.circle(surface, (255, 150, 0), to_screen(Bi), 5)
            pygame.draw.line(surface, (0, 200, 0), to_screen(Bi), to_screen(Ei), 3)
            pygame.draw.circle(surface, (255, 0, 0), to_screen(Ei), 5)
            triangle_points.append(to_screen(Ei))
        pygame.draw.polygon(surface, (0, 0, 0), triangle_points, 2)
    
    def draw_info(self, surface):
        info_text = f"Pose: x={self.pose[0]:.1f}, y={self.pose[1]:.1f}, theta={math.degrees(self.pose[2]):.1f}°"
        text_surf = font.render(info_text, True, (0, 0, 0))
        surface.blit(text_surf, (10, 10))
        mode_text = "Mode restreint: ON" if self.restricted_mode else "Mode restreint: OFF"
        mode_surf = font.render(mode_text, True, (0, 0, 0))
        surface.blit(mode_surf, (width - 180, 90))
        dessin_text = "Mode dessin: ON" if drawing_mode else "Mode dessin: OFF"
        dessin_surf = font.render(dessin_text, True, (0, 0, 0))
        surface.blit(dessin_surf, (width - 180, 110))
        # N'affiche pas le message "position limite atteinte"
        if self.error_msg and "limite" not in self.error_msg.lower():
            err_surf = font.render("Erreur: " + self.error_msg, True, (200, 0, 0))
            surface.blit(err_surf, (10, 30))

# -------- Calcul de la zone atteignable (une fois) --------
reachable_points = []
step = 10
for x in range(-300, 301, step):
    for y in range(-250, 251, step):
        try:
            Robot3RRR(L1, L2, Rb, Re, ang1, ang2).compute_kinematics((x, y, 0))
            reachable_points.append((x, y))
        except:
            continue

# -------- Variables de contrôle --------
robot = Robot3RRR(L1, L2, Rb, Re, ang1, ang2)
robot.restricted_mode = False  # Mode restreint par défaut OFF
drawing_mode = False           # Mode dessin OFF par défaut

current_pose = np.array([0.0, 0.0, 0.0])
target_pose  = np.array([0.0, 0.0, 0.0])
last_valid_pose = current_pose.copy()
try:
    robot.set_pose(tuple(current_pose))
except:
    pass

# -------- Pour enregistrer la trajectoire du centre de l'effecteur --------
trajectory = []  # liste des points tracés

# -------- Boucle principale --------
running = True
while running:
    screen.fill((255, 255, 255))
    
    # Afficher la zone atteignable
    for pt in reachable_points:
        pygame.draw.circle(screen, (220, 220, 220), to_screen(pt), 2)
    
    # Boutons Reset et Terminé
    draw_reset_button(screen)
    draw_termine_button(screen)
    
    # Affichage des instructions et infos
    draw_instructions(screen)
    #draw_mode_status(screen, robot.restricted_mode, drawing_mode)
    
    # Gestion du clavier pour modifier la cible
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        target_pose[0] -= delta_pos
    if keys[pygame.K_RIGHT]:
        target_pose[0] += delta_pos
    if keys[pygame.K_UP]:
        target_pose[1] += delta_pos
    if keys[pygame.K_DOWN]:
        target_pose[1] -= delta_pos
    if keys[pygame.K_q]:
        target_pose[2] += delta_theta
    if keys[pygame.K_d]:
        target_pose[2] -= delta_theta
    if keys[pygame.K_r]:
        target_pose = np.array([0.0, 0.0, 0.0])
    if keys[pygame.K_m]:
        robot.restricted_mode = not robot.restricted_mode
        pygame.time.wait(200)  # anti-rebond
    if keys[pygame.K_z]:
        drawing_mode = not drawing_mode
        pygame.time.wait(200)
    
    # Gestion de la souris pour les boutons
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if reset_button_rect.collidepoint(event.pos):
                target_pose = np.array([0.0, 0.0, 0.0])
            if termine_button_rect.collidepoint(event.pos):
                running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
    
    # Interpolation fluide vers la cible
    alpha_interp = 0.2
    current_pose = current_pose * (1 - alpha_interp) + target_pose * alpha_interp
    
    # Mise à jour de la pose du robot
    try:
        robot.set_pose(tuple(current_pose))
        last_valid_pose = current_pose.copy()
    except Exception as e:
        try:
            robot.set_pose(tuple(last_valid_pose))
        except:
            pass
        current_pose = last_valid_pose.copy()
    
    # Ajout de la trajectoire (le point médian de l'effecteur)
    try:
        q = robot.compute_kinematics(robot.pose)
        # Calcul du centre du triangle effecteur
        pts_effecteur = [item[3] for item in q]  # les Ei
        center_effecteur = (sum(p[0] for p in pts_effecteur) / 3,
                            sum(p[1] for p in pts_effecteur) / 3)
        trajectory.append(center_effecteur)
    except:
        pass
    
    # Dessiner la trajectoire (en bleu)
    if drawing_mode and len(trajectory) > 1:
        pygame.draw.lines(screen, (0, 0, 255), False, [to_screen(p) for p in trajectory], 2)
    
    # Dessiner le robot
    robot.draw(screen)
    robot.draw_info(screen)
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()