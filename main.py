import pygame
import pygame_gui
import math
import numpy as np
import math
# Initialize Pygame
pygame.init()

def pure_pursuit(ld,L,closest_point,vehicle_x,vehicle_y,rotation):
    delta_x = closest_point[0] - vehicle_x - 10
    delta_y = vehicle_y - 5 - closest_point[1]
    delta_x_t = delta_x * math.cos(math.radians(rotation)) + delta_y*math.cos(math.radians(rotation))
    delta_y_t = -delta_x * math.sin(math.radians(rotation)) + delta_y * math.cos(math.radians(rotation))
    alpha = math.atan2(delta_y_t,delta_x_t)
    u = math.atan((2*L*math.sin(alpha))/ld)
    print(u)
    return u
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
    value_range=(0, 200),
    manager=manager
)


# Set up the vehicle (a long rectangle with four smaller rectangles for wheels)
vehicle_color = (0, 0, 255)
wheel_color = (0, 0, 0)
vehicle_size = (30, 12)
wheel_size = (9, 5)

# Initial vehicle position
vehicle_x = 400
vehicle_y = 300

def closest_point(vehicle_x,vehicle_y,ld,path,last_idx,G):
    center = np.array([vehicle_x - 10, vehicle_y-5])
    radius = ld

    # Generate a NumPy array of points (example)
    if last_idx != -1:
        points = np.array(path[0:last_idx+350]) # 100 random points in a 10x10 grid
    else:
        points = np.array(path)
    # Calculate the distance of each point to the circle's center
    distances = np.linalg.norm(points - center, axis=1)


    tolerance = 15

    # Find the points that intersect the circle (distance approximately equal to the radius)
    intersection_mask = np.abs(distances - radius) < tolerance
    intersecting_points = points[intersection_mask]
    indexes = np.nonzero(intersection_mask)[0]
    if intersecting_points.size > 0:
        return intersecting_points[-1], indexes[-1], points
    else:
        return G,-1,points
def kinematic_bicecycle_sim(time_delta,vehicle_x,vehicle_y,theta,velocity,u,L=20,a=0):
    vehicle_x += velocity * math.cos(math.radians(theta)) * time_delta
    vehicle_y += -velocity * math.sin(math.radians(theta)) * time_delta
    theta = math.radians(theta) + velocity * math.tan(u)/L * time_delta
    velocity += 0

    return vehicle_x,vehicle_y,math.degrees(theta),velocity


# Function to draw the vehicle
def draw_vehicle(surface, color, position, size, rotation):
    # Create a surface for the vehicle
    vehicle_surf = pygame.Surface(size, pygame.SRCALPHA)
    vehicle_surf.fill((0, 0, 0, 0))
    pygame.draw.rect(vehicle_surf, color, (0, 0, *size))

    # Rotate the vehicle surface
    rotated_vehicle = pygame.transform.rotate(vehicle_surf, rotation)
    rotated_vehicle_rect = rotated_vehicle.get_rect(center=position)
    surface.blit(rotated_vehicle, rotated_vehicle_rect.topleft)

    # Calculate wheel positions relative to the vehicle center
    offset_x, offset_y = size[0] // 2, size[1] // 2
    wheel_positions = [
        (offset_x - 5, offset_y - 1),   # front left
        (offset_x - 25, offset_y - 1),   # read right
        (offset_x - 5, offset_y - 11),  # front left
        (offset_x - 25, offset_y - 11)   # rear left
    ]

    for pos in wheel_positions:
        # Create a surface for the wheel
        wheel_surf = pygame.Surface(wheel_size, pygame.SRCALPHA)
        wheel_surf.fill(wheel_color)
        # Rotate the wheel surface
        rotated_wheel = pygame.transform.rotate(wheel_surf, rotation)

        # Calculate wheel position after rotation
        wheel_x = pos[0] * math.cos(math.radians(rotation)) - pos[1] * math.sin(math.radians(rotation))
        wheel_y = pos[0] * math.sin(math.radians(rotation)) + pos[1] * math.cos(math.radians(rotation))
        wheel_center = (position[0] + wheel_x, position[1] - wheel_y)

        # Draw the rotated wheel
        rotated_wheel_rect = rotated_wheel.get_rect(center=wheel_center)
        surface.blit(rotated_wheel, rotated_wheel_rect.topleft)


def draw_random_open_curve(surface, window_size):
    num_points = 5000
    degree = 3

    # Generate random polynomial coefficients for x and y coordinates
    x_coefficients = np.random.uniform(-0.2, 0.2, size=degree + 1)
    y_coefficients = np.random.uniform(-0.2, 0.2, size=degree + 1)

    # Generate points using the polynomial coefficients
    t = np.linspace(0, 1.5 * np.pi, num_points)
    x_points = np.polyval(x_coefficients, np.cos(t))
    y_points = np.polyval(y_coefficients, np.sin(t))

    # Scale and shift points to fit window
    x_points = (x_points - np.min(x_points)) / (np.max(x_points) - np.min(x_points)) * (window_size[0] - 100) + 50
    y_points = (y_points - np.min(y_points)) / (np.max(y_points) - np.min(y_points)) * (window_size[1] - 100) + 50

    # Draw curve
    points = [(int(x), int(y)) for x, y in zip(x_points, y_points)]

    return points

# Main loop
running = True
last_idx = -1
clock = pygame.time.Clock()
loop_points = []
rotation = 0
velocity = 35
l_d = 5
u = 0
G = (-1,-1)
analyzed_points = []
while running:
    time_delta = clock.tick(60) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        manager.process_events(event)

    manager.update(time_delta)

    # Get the current values of the sliders
    velocity = slider_velocity.get_current_value()


    # Clear the window
    window.fill((255, 255, 255))
    l_d = np.clip(0.1 * velocity, 2, 400) + 15
    # Draw the vehicle
    if not loop_points:
        loop_points = draw_random_open_curve(window, window_size)
        vehicle_x = loop_points[0][0]
        vehicle_y = loop_points[0][1]
        start_rotation = math.atan2(loop_points[10][1] - loop_points[0][1],loop_points[10][0] - loop_points[0][0])
        rotation = math.degrees(-start_rotation)
        print(start_rotation)
    else:
        G,last_idx,analyzed_points = closest_point(vehicle_x,vehicle_y,l_d,loop_points,last_idx,G)
        u = pure_pursuit(l_d, 20, G, vehicle_x, vehicle_y,rotation)
        pygame.draw.circle(window, (0,255,0), G, 5)
        pygame.draw.lines(window, (255, 0, 0), False, loop_points, 2)
        pygame.draw.lines(window, (0, 255, 0), False, analyzed_points, 2)
    vehicle_x, vehicle_y, rotation, velocity = kinematic_bicecycle_sim(time_delta, vehicle_x, vehicle_y, rotation, velocity, u)
    draw_vehicle(window, vehicle_color, (vehicle_x, vehicle_y), vehicle_size, rotation)
    # Draw the GUI
    manager.draw_ui(window)

    pygame.display.flip()

pygame.quit()
