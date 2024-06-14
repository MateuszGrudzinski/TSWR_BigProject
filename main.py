import pygame
import pygame_gui
import numpy as np
import math

# Initialize Pygame
pygame.init()


# Pure Pursuit function for steering control
def pure_pursuit(ld, L, closest_point, vehicle_x, vehicle_y, rotation):
    delta_x = closest_point[0] - vehicle_x - 10
    delta_y = vehicle_y - 5 - closest_point[1]
    delta_x_t = delta_x * math.cos(math.radians(rotation)) + delta_y * math.cos(math.radians(rotation))
    delta_y_t = -delta_x * math.sin(math.radians(rotation)) + delta_y * math.cos(math.radians(rotation))
    alpha = math.atan2(delta_y_t, delta_x_t)
    u = math.atan((2 * L * math.sin(alpha)) / ld)
    return u


# Function to find the closest point on the path to the vehicle
def closest_point(vehicle_x, vehicle_y, ld, path, last_idx, G):
    center = np.array([vehicle_x - 10, vehicle_y - 5])
    radius = ld

    if last_idx != -1:
        points = np.array(path[0:last_idx + 250])
    else:
        points = np.array(path)

    distances = np.linalg.norm(points - center, axis=1)
    tolerance = 15
    intersection_mask = np.abs(distances - radius) < tolerance
    intersecting_points = points[intersection_mask]
    indexes = np.nonzero(intersection_mask)[0]

    if intersecting_points.size > 0:
        return intersecting_points[-1], indexes[-1], points
    else:
        return G, last_idx, points


# Kinematic bicycle model simulation function
def kinematic_bicycle_sim(time_delta, vehicle_x, vehicle_y, theta, velocity, u, L=20, a=0):
    vehicle_x += velocity * math.cos(math.radians(theta)) * time_delta
    vehicle_y += -velocity * math.sin(math.radians(theta)) * time_delta
    theta = math.radians(theta) + velocity * math.tan(u) / L * time_delta
    velocity += 0
    return vehicle_x, vehicle_y, math.degrees(theta), velocity


# Dynamic bicycle model simulation function
def dynamic_bicycle_sim(state, params, u):
    vy, r, theta, x, y = state
    theta = math.radians(theta)

    m, Iz, L, Caf, Car, vx = params
    Lf = Lr = L / 2

    A = -((Caf * math.cos(u) + Car) / (m * vx))
    B = ((-Lf * Caf * math.cos(u) + Lr * Car) / (m * vx)) - vx
    C = ((-Lf * Caf * math.cos(u) + Lr * Car)) / (Iz * vx)
    D = -((Lf ** 2 * Caf * math.cos(u) + Lr ** 2 * Car) / (Iz * vx))
    E = (Caf * math.cos(u)) / m
    F = (Lf * Caf * math.cos(u)) / Iz

    vy_dot = A * vy + C * r * E * u
    r_dot = B * vy + D * r + F * u
    x_dot = vx * math.cos(theta) + vy * math.sin(theta)
    y_dot = -(vx * math.sin(theta) - vy * math.cos(theta))
    theta_dot = r

    return np.array([vy_dot, r_dot, math.degrees(theta_dot), x_dot, y_dot])


# Runge-Kutta method for updating the state of the vehicle
def incremental_update(state, params, delta_f, dt):
    k1 = dt * dynamic_bicycle_sim(state, params, delta_f)
    k2 = dt * dynamic_bicycle_sim(state + 0.5 * k1, params, delta_f)
    k3 = dt * dynamic_bicycle_sim(state + 0.5 * k2, params, delta_f)
    k4 = dt * dynamic_bicycle_sim(state + k3, params, delta_f)

    new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return new_state


# Function to draw the vehicle on the screen
def draw_vehicle(surface, color, position, size, rotation):
    vehicle_surf = pygame.Surface(size, pygame.SRCALPHA)
    vehicle_surf.fill((0, 0, 0, 0))
    pygame.draw.rect(vehicle_surf, color, (0, 0, *size))

    rotated_vehicle = pygame.transform.rotate(vehicle_surf, rotation)
    rotated_vehicle_rect = rotated_vehicle.get_rect(center=position)
    surface.blit(rotated_vehicle, rotated_vehicle_rect.topleft)

    offset_x, offset_y = size[0] // 2, size[1] // 2
    wheel_positions = [
        (offset_x - 5, offset_y - 1),
        (offset_x - 25, offset_y - 1),
        (offset_x - 5, offset_y - 11),
        (offset_x - 25, offset_y - 11)
    ]

    for pos in wheel_positions:
        wheel_surf = pygame.Surface((9, 5), pygame.SRCALPHA)
        wheel_surf.fill((0, 0, 0))
        rotated_wheel = pygame.transform.rotate(wheel_surf, rotation)

        wheel_x = pos[0] * math.cos(math.radians(rotation)) - pos[1] * math.sin(math.radians(rotation))
        wheel_y = pos[0] * math.sin(math.radians(rotation)) + pos[1] * math.cos(math.radians(rotation))
        wheel_center = (position[0] + wheel_x, position[1] - wheel_y)

        rotated_wheel_rect = rotated_wheel.get_rect(center=wheel_center)
        surface.blit(rotated_wheel, rotated_wheel_rect.topleft)


# Function to generate a random open curve
def draw_random_open_curve(window_size):
    num_points = 5000
    degree = 2

    x_coefficients = np.random.uniform(-0.05, 0.05, size=degree + 1)
    y_coefficients = np.random.uniform(-0.05, 0.05, size=degree + 1)

    t = np.linspace(0, np.pi, num_points)
    x_points = np.polyval(x_coefficients, np.cos(t))
    y_points = np.polyval(y_coefficients, np.sin(t))

    x_points = (x_points - np.min(x_points)) / (np.max(x_points) - np.min(x_points)) * (window_size[0] - 100) + 50
    y_points = (y_points - np.min(y_points)) / (np.max(y_points) - np.min(y_points)) * (window_size[1] - 100) + 50

    points = [(int(x), int(y)) for x, y in zip(x_points, y_points)]
    return points


# Set up the display
window_size = (800, 600)
window = pygame.display.set_mode(window_size)
pygame.display.set_caption('Control Vehicle with Sliders')

# Set up the GUI manager
manager = pygame_gui.UIManager(window_size)

# Set up labels
label_velocity = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((550, 20), (200, 30)),
    text='Velocity',
    manager=manager
)

# Set up sliders
slider_velocity = pygame_gui.elements.ui_horizontal_slider.UIHorizontalSlider(
    relative_rect=pygame.Rect((550, 50), (200, 30)),
    start_value=0,
    value_range=(100, 200),
    manager=manager
)

# Vehicle properties
vehicle_color = (0, 0, 255)
vehicle_color_k = (0, 255, 255)
vehicle_size = (30, 12)

# Initial vehicle position
vehicle_x, vehicle_y = 400, 300
velocity = 35

# Main loop
running = True
last_idx = -1
last_idx_k = -1
clock = pygame.time.Clock()
loop_points = []
rotation = 0
velocity = 35
velocity_k = 35
l_d = 5
l_d_k = 10
u = 0
u_k = 0
G = (-1,-1)
G_k = (-1,-1)
analyzed_points = []
elapsed_time = 0
while running:
    time_delta = clock.tick(60) / 1000.0
    elapsed_time += time_delta
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        manager.process_events(event)

    manager.update(time_delta)

    # Get the current values of the sliders
    velocity = slider_velocity.get_current_value()


    # Clear the window
    window.fill((255, 255, 255))
    l_d_k = np.clip(0.20 * velocity, 25, 200)
    l_d = np.clip(0.35 * velocity, 25, 200)
    # Draw the vehicle
    if not loop_points:
        loop_points = draw_random_open_curve(window_size)
        vehicle_x = loop_points[0][0]
        vehicle_y = loop_points[0][1]
        vehicle_x_k = loop_points[0][0]
        vehicle_y_k = loop_points[0][1]
        start_rotation = math.atan2(loop_points[10][1] - loop_points[0][1],loop_points[10][0] - loop_points[0][0])
        rotation = math.degrees(-start_rotation)
        rotation_k = math.degrees(-start_rotation)
        initial_state = [0.00, 0.0, rotation, vehicle_x, vehicle_y]
        state = initial_state
    else:
        G,last_idx,analyzed_points = closest_point(vehicle_x,vehicle_y,l_d,loop_points,last_idx,G)
        u = pure_pursuit(l_d, 20, G, vehicle_x, vehicle_y,rotation)
        G_k, last_idx_k, analyzed_points_k = closest_point(vehicle_x_k, vehicle_y_k, l_d_k, loop_points, last_idx_k, G_k)
        u_k = pure_pursuit(l_d_k, 20, G_k, vehicle_x_k, vehicle_y_k, rotation_k)
        pygame.draw.circle(window, (0,0,255), G, 5)
        pygame.draw.circle(window, (0, 255, 255), G_k, 5)
        pygame.draw.lines(window, (255, 0, 0), False, loop_points, 2)
        pygame.draw.lines(window, (0, 255, 0), False, analyzed_points, 4)
    vehicle_x_k, vehicle_y_k, rotation_k, velocity_k = kinematic_bicycle_sim(time_delta, vehicle_x_k, vehicle_y_k, rotation_k, velocity, u_k)
    params = [300, 5, 20, 50, 50, velocity]
    state = incremental_update(state, params, u, time_delta)
    vehicle_x,vehicle_y,rotation = state[3],state[4],state[2]
    print(state[0])
    draw_vehicle(window, vehicle_color, (vehicle_x, vehicle_y), vehicle_size, rotation)
    draw_vehicle(window, vehicle_color_k, (vehicle_x_k, vehicle_y_k), vehicle_size, rotation_k)
    # Draw the GUI
    manager.draw_ui(window)

    pygame.display.flip()
    if elapsed_time > 15 or ((last_idx == len(loop_points) - 1) and (last_idx_k == len(loop_points) - 1)):
        last_idx = -1
        last_idx_k = -1
        elapsed_time = 0
        loop_points = []
pygame.quit()