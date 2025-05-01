import math

# Dimensions du robot (en mètres)
L1 = 0.10        # Longueur bras  1
L2 = 0.10        # Longueur bras  2 
Rb = 0.1322594   # Rayon de la base  
Re = 0.07        # Rayon de l’effecteur   


# Affichage Pygame
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# Couleurs RGB
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (180, 180, 180)
BLUE = (50, 100, 255)
RED = (255, 0, 0)
GREEN = (0, 200, 0)
YELLOW = (255, 255, 0)

COLOR_ROBOT = BLUE
COLOR_EFFECTOR = GREEN
COLOR_TRAJECTORY = YELLOW
COLOR_ERROR = RED

#⏱️ Simulation
SPEED = 0.01         # Vitesse déplacement en mètre/frame
THETA_SPEED = math.radians(1.5)  # Rotation rad/frame

# Seuil de détection de singularité
SINGULARITY_THRESHOLD = 1e-4

# Une configuration est singulière si det(A) = 0 ou det(B) = 0
# Mais dans notre simulateur, det(A) sera parfois juste très proche de zéro, par exemple det(A) = 0.000002 en pratique on la considere comme une singulartité 


# Position initiale de l’effecteur (en mètres)
INIT_XE = 0.0
INIT_YE = 0.0
INIT_THETAE = 0.0  # radians

# Trajectoire
DRAW_TRAJECTORY = True
