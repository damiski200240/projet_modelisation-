import numpy as np
import math

def generate_circle(center, radius, n_points):
    cx, cy = center
    return [(cx + radius * math.cos(2 * math.pi * i / n_points),
             cy + radius * math.sin(2 * math.pi * i / n_points),
             0.0) for i in range(n_points)]

def generate_square(center, side, n_points):
    cx, cy = center
    half = side / 2
    pts = [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half)
    ]
    result = []
    for i in range(4):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        for j in range(n_points // 4):
            t = j / (n_points // 4)
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            result.append((x, y, 0.0))
    return result

def generate_star(center, size, n_points):
    cx, cy = center
    result = []
    for i in range(n_points):
        angle = 2 * math.pi * i / n_points
        r = size if i % 2 == 0 else size / 2
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        result.append((x, y, 0.0))
    result.append(result[0])  # boucle fermée
    return result

def generate_spiral(center, max_radius, n_turns, n_points):
    cx, cy = center
    result = []
    for i in range(n_points):
        t = i / n_points
        angle = 2 * math.pi * n_turns * t
        radius = max_radius * t
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        result.append((x, y, angle))
    return result

def get_trajectory_by_name(name, center, size, n_points=100):
    name = name.lower()
    if name == "cercle":
        return generate_circle(center, size, n_points)
    elif name == "carre":
        return generate_square(center, size, n_points)
    elif name == "etoile":
        return generate_star(center, size, n_points)
    elif name == "spirale":
        return generate_spiral(center, size, 3, n_points)
    else:
        return []
