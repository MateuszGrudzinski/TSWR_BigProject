import pygame
import pygame_gui
import math
import numpy as np
import math
# Initialize Pygame
pygame.init()

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

label_rotation = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((550, 80), (200, 30)),
    text='Rotation (degrees)',
    manager=manager
)

# Set up sliders
slider_velocity = pygame_gui.elements.ui_horizontal_slider.UIHorizontalSlider(
    relative_rect=pygame.Rect((550, 50), (200, 30)),
    start_value=0,
    value_range=(-100, 100),
    manager=manager
)

slider_rotation = pygame_gui.elements.ui_horizontal_slider.UIHorizontalSlider(
    relative_rect=pygame.Rect((550, 110), (200, 30)),
    start_value=0,
    value_range=(0, 360),
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
        (-offset_x + 5, offset_y - 1),   # Rear left
        (offset_x - 8, offset_y - 1),   # Rear right
        (-offset_x + 5, -offset_y + 1),  # Front left
        (offset_x - 8, -offset_y + 1)   # Front right
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


def draw_random_closed_curve(surface, window_size):
    num_points = 1000
    degree = np.random.randint(3, 4)

    # Generate random polynomial coefficients for x and y coordinates
    x_coefficients = np.random.uniform(-1, 1, size=degree + 1)
    y_coefficients = np.random.uniform(-1, 1, size=degree + 1)

    # Generate points using the polynomial coefficients
    t = np.linspace(0, 2 * np.pi, num_points)
    x_points = np.polyval(x_coefficients, np.cos(t))
    y_points = np.polyval(y_coefficients, np.sin(t))

    # Scale and shift points to fit window
    x_points = (x_points - np.min(x_points)) / (np.max(x_points) - np.min(x_points)) * (window_size[0] - 100) + 50
    y_points = (y_points - np.min(y_points)) / (np.max(y_points) - np.min(y_points)) * (window_size[1] - 100) + 50

    # Draw curve
    points = [(int(x), int(y)) for x, y in zip(x_points, y_points)]
    pygame.draw.lines(surface, (0, 0, 0), True, points, 2)  # 2 is the line width

    return points

# Main loop
running = True
clock = pygame.time.Clock()
loop_points = []
while running:
    time_delta = clock.tick(60) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        manager.process_events(event)

    manager.update(time_delta)

    # Get the current values of the sliders
    velocity = slider_velocity.get_current_value()
    rotation = slider_rotation.get_current_value()

    # Update vehicle position based on velocity and rotation
    vehicle_x += velocity * math.cos(math.radians(rotation)) * time_delta
    vehicle_y -= velocity * math.sin(math.radians(rotation)) * time_delta

    # Clear the window
    window.fill((255, 255, 255))

    # Draw the vehicle
    if not loop_points:
        loop_points = draw_random_closed_curve(window, window_size)
        vehicle_x = loop_points[0][0]
        vehicle_y = loop_points[0][1]
    else:
        # Draw the loop using stored points
        pygame.draw.lines(window, (0, 0, 0), True, loop_points, 2)
    draw_vehicle(window, vehicle_color, (vehicle_x, vehicle_y), vehicle_size, rotation)

    # Draw the GUI
    manager.draw_ui(window)

    pygame.display.flip()

pygame.quit()
