import pygame
import math

def handle_key_event(event, robot, pose, drawing_mode, input_mode):
    """
    Gère les appuis clavier pour interagir avec le robot.
    Retourne : (nouvelle pose, nouveau mode dessin, nouveau mode de saisie)
    """
    x, y, theta = pose
    step = 0.005       # mètre
    angle_step = math.radians(2)  # rad

    if event.type == pygame.KEYDOWN:
        # Contrôle position seulement si mode clavier
        if input_mode == "keyboard":
            if event.key == pygame.K_UP:
                y += step
            elif event.key == pygame.K_DOWN:
                y -= step
            elif event.key == pygame.K_LEFT:
                x -= step
            elif event.key == pygame.K_RIGHT:
                x += step
            elif event.key == pygame.K_a:
                theta += angle_step
            elif event.key == pygame.K_e:
                theta -= angle_step

        # Basculer méthode MGI
        if event.key == pygame.K_m:
            robot.mode = "numeric" if robot.mode == "analytic" else "analytic"
            print(f"[MODE MGI] => {robot.mode.upper()}")

        # Mode restreint
        elif event.key == pygame.K_r:
            robot.restricted_mode = not robot.restricted_mode
            print(f"[MODE RESTREINT] => {'ON' if robot.restricted_mode else 'OFF'}")

        # Mode dessin
        elif event.key == pygame.K_d:
            drawing_mode = not drawing_mode
            print(f"[MODE DESSIN] => {'ON' if drawing_mode else 'OFF'}")

        # Basculer mode saisie clavier / manuelle
        elif event.key == pygame.K_SPACE:
            input_mode = "manual" if input_mode == "keyboard" else "keyboard"
            print(f"[MODE CONTROLE] => {input_mode.upper()}")

    return (x, y, theta), drawing_mode, input_mode
