import pygame
import math
import numpy as np
from robot import Robot3RRR
from graphics import draw_robot, draw_info, draw_trajectory, draw_button, draw_instructions, to_screen
from config import L1, L2, Rb, Re, SCREEN_WIDTH, SCREEN_HEIGHT, FPS
from trajectoires import get as get_trajectory, names as trajectory_names

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Simulateur Robot 3RRR")
font = pygame.font.SysFont("Arial", 16)
clock = pygame.time.Clock()

# Initialisation du robot
ang1 = [0, 2 * math.pi / 3, 4 * math.pi / 3]
ang2 = [-math.pi / 2, math.pi / 6, 5 * math.pi / 6]
robot = Robot3RRR(L1, L2, Rb, Re, ang1, ang2)

pose = (0.0, 0.0, 0.0)
last_valid_pose = pose

drawing_mode = False
trajectory_points = []

# Saisie clavier / manuelle
input_mode = "keyboard"
input_active = False
input_text = ""
input_box = pygame.Rect(SCREEN_WIDTH - 220, 200, 180, 30)
input_color_active = (0, 200, 0)
input_color_inactive = (180, 180, 180)
input_color = input_color_inactive
saisie_step = 0
saisie_vals = [0.0, 0.0, 0.0]

# Trajectoires prédéfinies
available_trajectories = trajectory_names()
selected_index = 0
current_trajectory = []
trajectory_index = 0
follow_trajectory = False

# Boutons
reset_button_rect = pygame.Rect(SCREEN_WIDTH - 110, 10, 100, 30)
start_button_rect = pygame.Rect(SCREEN_WIDTH - 110, 50, 100, 30)
next_button_rect = pygame.Rect(SCREEN_WIDTH - 40, 90, 30, 30)
prev_button_rect = pygame.Rect(SCREEN_WIDTH - 150, 90, 30, 30)

# Espace atteignable
reachable_points = []
def compute_reachable_space():
    global reachable_points
    step = 0.005
    for x in np.arange(-0.2, 0.2, step):
        for y in np.arange(-0.2, 0.2, step):
            try:
                robot.set_pose((x, y, 0))
                if robot.q is not None:
                    reachable_points.append((x, y))
            except:
                continue
compute_reachable_space()

def draw_predefined_trajectory(screen, points):
    if len(points) > 1:
        pygame.draw.lines(screen, (255, 180, 0), False, [to_screen(p) for p in points], 1)

# -------- BOUCLE PRINCIPALE --------
running = True
while running:
    screen.fill((255, 255, 255))

    for pt in reachable_points:
        px = int(pt[0] * 1000 + SCREEN_WIDTH // 2)
        py = int(-pt[1] * 1000 + SCREEN_HEIGHT // 2)
        pygame.draw.circle(screen, (210, 240, 255), (px, py), 1)

    draw_button(screen, reset_button_rect, "RESET", font)
    draw_button(screen, start_button_rect, "START", font)
    draw_button(screen, next_button_rect, ">", font)
    draw_button(screen, prev_button_rect, "<", font)
    traj_label = f"Traj : {available_trajectories[selected_index]}"
    screen.blit(font.render(traj_label, True, (0, 0, 0)), (SCREEN_WIDTH - 220, 95))
    draw_instructions(screen, font, SCREEN_HEIGHT)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_TAB or event.key == pygame.K_SPACE:
                input_mode = "manual" if input_mode == "keyboard" else "keyboard"
                input_text = ""
                saisie_step = 0
                saisie_vals = [0.0, 0.0, 0.0]
                input_active = True
                input_color = input_color_active

            elif event.key == pygame.K_c:
                trajectory_points.clear()
                follow_trajectory = False
                trajectory_index = 0
                print("[INFO] Trajectoire effacée.")

            elif event.key == pygame.K_d:
                drawing_mode = not drawing_mode

            elif event.key == pygame.K_m:
                robot.mode = "numeric" if robot.mode == "analytic" else "analytic"

            elif event.key == pygame.K_r:
                pose = (0.0, 0.0, 0.0)
                input_text = ""
                trajectory_points.clear()
                follow_trajectory = False
                trajectory_index = 0

            elif event.key == pygame.K_p:
                robot.restricted_mode = not robot.restricted_mode

            if input_mode == "manual" and input_active:
                if event.key == pygame.K_RETURN:
                    try:
                        val = float(input_text.strip())
                        saisie_vals[saisie_step] = val
                        input_text = ""
                        saisie_step += 1
                        if saisie_step > 2:
                            x, y, theta_deg = saisie_vals
                            pose = (x, y, math.radians(theta_deg))
                            saisie_step = 0
                            saisie_vals = [0.0, 0.0, 0.0]
                    except:
                        input_text = ""
                elif event.key == pygame.K_BACKSPACE:
                    input_text = input_text[:-1]
                else:
                    input_text += event.unicode

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if input_box.collidepoint(event.pos):
                input_active = True
            else:
                input_active = False
            input_color = input_color_active if input_active else input_color_inactive

            if reset_button_rect.collidepoint(event.pos):
                pose = (0.0, 0.0, 0.0)
                input_text = ""
                trajectory_points.clear()
                follow_trajectory = False
                trajectory_index = 0

            elif start_button_rect.collidepoint(event.pos):
                current_trajectory = get_trajectory(available_trajectories[selected_index])
                trajectory_index = 0
                follow_trajectory = True
                drawing_mode = True

            elif next_button_rect.collidepoint(event.pos):
                selected_index = (selected_index + 1) % len(available_trajectories)

            elif prev_button_rect.collidepoint(event.pos):
                selected_index = (selected_index - 1) % len(available_trajectories)

    if input_mode == "keyboard":
        keys = pygame.key.get_pressed()
        dx = dy = dtheta = 0
        step = 0.002
        rot_step = math.radians(1)
        if keys[pygame.K_LEFT]: dx -= step
        if keys[pygame.K_RIGHT]: dx += step
        if keys[pygame.K_UP]: dy += step
        if keys[pygame.K_DOWN]: dy -= step
        if keys[pygame.K_a]: dtheta += rot_step
        if keys[pygame.K_e]: dtheta -= rot_step
        pose = (pose[0] + dx, pose[1] + dy, pose[2] + dtheta)

    if follow_trajectory and trajectory_index < len(current_trajectory):
        x, y = current_trajectory[trajectory_index]
        pose = (x, y, 0.0)
        trajectory_index += 1
    elif follow_trajectory:
        follow_trajectory = False

    try:
        robot.set_pose(pose)
        if robot.q is not None:
            last_valid_pose = pose
    except:
        pose = last_valid_pose
        try:
            robot.set_pose(pose)
        except:
            pass

    if drawing_mode and robot.q is not None:
        center = (pose[0], pose[1])
        if not trajectory_points or trajectory_points[-1] != center:
            trajectory_points.append(center)

    draw_predefined_trajectory(screen, current_trajectory)
    draw_trajectory(screen, trajectory_points)
    draw_robot(screen, robot)
    draw_info(screen, robot, font, SCREEN_WIDTH, drawing_mode)

    mode_saisie = f"Contrôle: {'CLAVIER' if input_mode == 'keyboard' else 'SAISIE'} | MGI: {robot.mode.upper()}"
    screen.blit(font.render(mode_saisie, True, (0, 0, 0)), (10, SCREEN_HEIGHT - 30))

    if input_mode == "manual":
        pygame.draw.rect(screen, input_color, input_box, 2)
        labels = ["x", "y", "θ°"]
        prompt = f"{labels[saisie_step]} = {input_text}"
        txt_surface = font.render(prompt, True, (0, 0, 0))
        screen.blit(txt_surface, (input_box.x + 10, input_box.y + 5))

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
