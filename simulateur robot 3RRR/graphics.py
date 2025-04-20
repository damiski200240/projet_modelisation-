import pygame
import math

def to_screen(pos):
    scale = 1000
    offset = (400, 300)
    return int(pos[0] * scale + offset[0]), int(-pos[1] * scale + offset[1])

def draw_robot(screen, robot):
    robot.draw(screen)

def draw_info(screen, robot, font, width, drawing_mode):
    robot.draw_info(screen, font, width, drawing_mode)

def draw_trajectory(screen, points):
    if len(points) > 1:
        pygame.draw.lines(screen, (100, 100, 255), False, [to_screen(p) for p in points], 2)

def draw_button(surface, rect, label, font):
    pygame.draw.rect(surface, (200, 200, 200), rect)
    pygame.draw.rect(surface, (100, 100, 100), rect, 2)
    text = font.render(label, True, (0, 0, 0))
    surface.blit(text, text.get_rect(center=rect.center))

def draw_instructions(surface, font, height):
    lines = [
        "Flèches : déplacement (x, y)",
        "A / E : rotation θ",
        "R : Reset position",
        "M : Mode MGI",
        "D : Mode dessin ON/OFF",
        "TAB ou ESPACE : clavier/saisie",
        "C : Effacer",
        "P : Mode restreint ON/OFF",
        "ESC : Quitter"
    ]
    for i, line in enumerate(lines):
        txt = font.render(line, True, (0, 0, 0))
        surface.blit(txt, (10, 120 + 20 * i))
