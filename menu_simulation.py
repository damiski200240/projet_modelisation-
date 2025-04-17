import pygame
from trajectoires import get_trajectory_by_name

def menu_selection():
    pygame.init()
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Menu Robot 3RRR")
    font = pygame.font.SysFont("Arial", 26)

    class Button:
        def __init__(self, rect, text, callback):
            self.rect = pygame.Rect(rect)
            self.text = text
            self.callback = callback

        def draw(self, surface):
            pygame.draw.rect(surface, (180, 180, 180), self.rect)
            pygame.draw.rect(surface, (0, 0, 0), self.rect, 2)
            label = font.render(self.text, True, (0, 0, 0))
            label_rect = label.get_rect(center=self.rect.center)
            surface.blit(label, label_rect)

        def handle_event(self, event):
            if event.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(event.pos):
                self.callback()

    selected_shape = None
    ready = False
    trajectory = []

    center = (0, 0)
    size = 100

    def select_shape(name):
        nonlocal selected_shape
        selected_shape = name

    def launch():
        nonlocal ready, trajectory
        if selected_shape:
            trajectory = get_trajectory_by_name(selected_shape, center, size, 100)
            ready = True

    def quit_program():
        pygame.quit()
        exit()

    buttons = [
        Button((100, 100, 200, 50), "Cercle", lambda: select_shape("cercle")),
        Button((100, 170, 200, 50), "Carré", lambda: select_shape("carre")),
        Button((100, 240, 200, 50), "Étoile", lambda: select_shape("etoile")),
        Button((100, 310, 200, 50), "Spirale", lambda: select_shape("spirale")),
        Button((400, 150, 250, 60), "Lancer le dessin", launch),
        Button((400, 240, 250, 60), "Quitter", quit_program)
    ]

    while not ready:
        screen.fill((255, 255, 255))
        title = font.render("Choisissez une forme à dessiner", True, (0, 0, 0))
        screen.blit(title, (100, 30))
        for btn in buttons:
            btn.draw(screen)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_program()
            for btn in buttons:
                btn.handle_event(event)
        pygame.display.flip()

    pygame.quit()
    return trajectory
