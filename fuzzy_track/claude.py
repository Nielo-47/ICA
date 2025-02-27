import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from skfuzzy import control as ctrl
import pygame
import pandas as pd
import time
import sys

# Initialize pygame
pygame.init()
WIDTH, HEIGHT = 1200, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Fuzzy Car Racing Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (200, 200, 200)

# Font
font = pygame.font.SysFont("Arial", 18)


def generate_race_track():
    # Define key control points for the track (mix of curves and straights)
    control_points = (
        np.array(
            [
                [0, 0],
                [2, 1],
                [4, 1],  # First straight
                [6, 2],
                [7, 5],
                [6, 6],  # First curve
                [4, 7],
                [2, 7],
                [0, 6],  # Second straight
                [-2, 5],
                [-4, 4],
                [-2, 1],  # Second curve
                [0, 0],  # Close the loop
            ]
        ).T
        * 10
    )  # Transpose for compatibility

    # Generate a smooth spline curve through control points
    tck, u = splprep(control_points, s=0, per=True)  # Closed curve
    u_fine = np.linspace(0, 1, 500)  # Increased number of points for smoothness
    smooth_track = splev(u_fine, tck)

    return smooth_track[:2]


def generate_road_limits(race_track, width=4):
    """
    Gera os limites interno e externo da estrada com largura constante.
    """
    x, y = race_track

    # Calcular a direção tangente à pista
    dx = np.gradient(x)
    dy = np.gradient(y)

    # Calcular a direção perpendicular (vetor normal)
    norm = np.sqrt(dx**2 + dy**2)
    nx = -dy / norm
    ny = dx / norm

    # Calcular os limites interno e externo
    x_inner = x - nx * width / 2
    y_inner = y - ny * width / 2
    x_outer = x + nx * width / 2
    y_outer = y + ny * width / 2

    return np.column_stack((x_inner, y_inner, x_outer, y_outer))


class FuzzyCar:
    def __init__(self, width, length, max_speed=10, start_pos=None, start_angle=None):
        self.width = width
        self.length = length
        self.max_speed = max_speed
        self.curr_speed = 0

        # Initialize with provided start position and angle or defaults
        self.curr_angle = start_angle if start_angle is not None else -np.pi / 2
        self.current_pos = start_pos if start_pos is not None else (0, 0)

        self.state = pd.DataFrame(
            [[0, 0, 0, 0, *self.velocity()]],
            columns=["Proximity Diff.", "Delta Diff.", "Delta Angle", "Acceleration", "Speed", "Angle"],
        )
        self.laps = 0
        self.last_checkpoint = 0
        self.checkpoints_passed = set()
        self.create_fuzzy_system()

    def create_fuzzy_system(self):
        proximity_diff = ctrl.Antecedent(np.arange(-0.5, 0.51, 0.001), "proximity_diff")
        delta_diff = ctrl.Antecedent(np.arange(-0.05, 0.051, 0.0001), "delta_diff")
        acceleration = ctrl.Consequent(np.arange(-0.1, 0.1, 0.001), "acceleration")

        # Positivo: margem direita mais próxima
        proximity_diff.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])
        delta_diff.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])

        # Positivo: sentido anti-horário
        max_radian = 15 * np.pi / 180  # 15 degrees in radians
        delta_angle = ctrl.Consequent(np.arange(-max_radian, max_radian, 0.001), "angle")

        delta_angle.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])
        acceleration.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])

        rules = [
            ctrl.Rule(proximity_diff["NG"] & delta_diff["NG"], (delta_angle["NG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NG"] & delta_diff["NM"], (delta_angle["NG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NG"] & delta_diff["NP"], (delta_angle["NG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NG"] & delta_diff["ZE"], (delta_angle["NG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NG"] & delta_diff["PP"], (delta_angle["NM"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NG"] & delta_diff["PM"], (delta_angle["NP"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NG"] & delta_diff["PG"], (delta_angle["ZE"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["NG"], (delta_angle["NG"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["NM"], (delta_angle["NG"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["NP"], (delta_angle["NG"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["ZE"], (delta_angle["NM"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["PP"], (delta_angle["NP"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["PM"], (delta_angle["ZE"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NM"] & delta_diff["PG"], (delta_angle["PP"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["NG"], (delta_angle["NG"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["NM"], (delta_angle["NG"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["NP"], (delta_angle["NM"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["ZE"], (delta_angle["NP"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["PP"], (delta_angle["ZE"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["PM"], (delta_angle["PP"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["NP"] & delta_diff["PG"], (delta_angle["PM"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["NG"], (delta_angle["NG"], acceleration["PM"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["NM"], (delta_angle["NM"], acceleration["PM"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["NP"], (delta_angle["NP"], acceleration["PG"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["ZE"], (delta_angle["ZE"], acceleration["PG"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["PP"], (delta_angle["PP"], acceleration["PG"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["PM"], (delta_angle["PM"], acceleration["PM"])),
            ctrl.Rule(proximity_diff["ZE"] & delta_diff["PG"], (delta_angle["PG"], acceleration["PM"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["NG"], (delta_angle["NM"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["NM"], (delta_angle["NP"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["NP"], (delta_angle["ZE"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["ZE"], (delta_angle["PP"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["PP"], (delta_angle["PM"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["PM"], (delta_angle["PG"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PP"] & delta_diff["PG"], (delta_angle["PG"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["NG"], (delta_angle["NP"], acceleration["PP"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["NM"], (delta_angle["ZE"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["NP"], (delta_angle["PP"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["ZE"], (delta_angle["PM"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["PP"], (delta_angle["PG"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["PM"], (delta_angle["PG"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["PM"] & delta_diff["PG"], (delta_angle["PG"], acceleration["NM"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["NG"], (delta_angle["ZE"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["NM"], (delta_angle["PP"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["NP"], (delta_angle["PM"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["ZE"], (delta_angle["PG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["PP"], (delta_angle["PG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["PM"], (delta_angle["PG"], acceleration["NG"])),
            ctrl.Rule(proximity_diff["PG"] & delta_diff["PG"], (delta_angle["PG"], acceleration["NG"])),
        ]

        # Criando o sistema de controle e simulacao
        control_system = ctrl.ControlSystem(rules)
        self.fuzzy_sim = ctrl.ControlSystemSimulation(control_system)

    def velocity(self):
        return np.array((self.curr_speed, self.curr_angle))

    def nearest_limit_points_diff(self, limits):
        inner_dists = np.sum(np.square(limits[:, :2] - self.current_pos), axis=1)
        outer_dists = np.sum(np.square(limits[:, 2:] - self.current_pos), axis=1)
        min_outer = np.sqrt(min(outer_dists))
        min_inner = np.sqrt(min(inner_dists))

        if min_outer > 4.5 or min_inner > 4.5:
            raise Exception("Fora da pista")

        return min_outer - min_inner

    def update_state(self, proximity_diff, delta_diff, delta_angle, accel):
        self.state.loc[len(self.state)] = np.array(
            [
                proximity_diff,
                delta_diff,
                delta_angle,
                accel,
                *self.velocity(),
            ]
        ).round(6)

    def update_position(self, limits, race_track):
        proximity_diff = self.nearest_limit_points_diff(limits)
        delta_diff = proximity_diff - self.state.values[-1][0]

        self.fuzzy_sim.input["proximity_diff"] = proximity_diff
        self.fuzzy_sim.input["delta_diff"] = delta_diff

        self.fuzzy_sim.compute()
        angle_delta = self.fuzzy_sim.output["angle"]
        accel = self.fuzzy_sim.output["acceleration"]

        self.update_state(proximity_diff, delta_diff, angle_delta, accel)

        self.curr_speed += accel
        self.curr_speed = np.clip(self.curr_speed, 0, self.max_speed)

        self.curr_angle += angle_delta

        # Ensure car moves forward based on current speed and angle
        dx = self.curr_speed * np.cos(self.curr_angle)
        dy = self.curr_speed * np.sin(self.curr_angle)
        self.current_pos = (self.current_pos[0] + dx, self.current_pos[1] + dy)

        # Check for lap completion (simplified)
        self.check_lap_progress(race_track)

        return self.current_pos

    def check_lap_progress(self, race_track):
        # Create checkpoints along the track
        num_checkpoints = 20
        checkpoint_indices = np.linspace(0, len(race_track[0]) - 1, num_checkpoints, dtype=int)

        # Find the nearest point on the track
        distances = [
            (self.current_pos[0] - race_track[0][i]) ** 2 + (self.current_pos[1] - race_track[1][i]) ** 2
            for i in range(len(race_track[0]))
        ]
        nearest_idx = np.argmin(distances)

        # Check if we're at a checkpoint
        for i, idx in enumerate(checkpoint_indices):
            # If we're close to this checkpoint
            if abs(nearest_idx - idx) < 100:
                if i not in self.checkpoints_passed:
                    self.checkpoints_passed.add(i)

        # If we've passed all checkpoints and are back at the start
        if len(self.checkpoints_passed) == num_checkpoints and abs(nearest_idx - 0) < 100:
            self.laps += 1
            self.checkpoints_passed = set()


def world_to_screen(pos, min_x, min_y, scale):
    """Convert world coordinates to screen coordinates"""
    screen_x = int((pos[0] - min_x) * scale)
    screen_y = int(HEIGHT - (pos[1] - min_y) * scale)
    return (screen_x, screen_y)


def draw_car(screen, pos, angle, scale, size=10):
    """Draw a simple car representation"""
    # Calculate the car's four corners
    car_length = size * 2
    car_width = size

    # Calculate the corners of the car based on angle and position
    corners = [
        (-car_length / 2, -car_width / 2),
        (car_length / 2, -car_width / 2),
        (car_length / 2, car_width / 2),
        (-car_length / 2, car_width / 2),
    ]

    # Rotate and translate corners
    rotated_corners = []
    for x, y in corners:
        # Rotate
        x_rot = x * np.cos(angle) - y * np.sin(angle)
        y_rot = x * np.sin(angle) + y * np.cos(angle)

        # Translate to car position
        screen_pos = (int(pos[0] + x_rot), int(pos[1] + y_rot))
        rotated_corners.append(screen_pos)

    # Draw the car
    pygame.draw.polygon(screen, RED, rotated_corners)

    # Draw direction indicator
    front_center = (int(pos[0] + car_length / 2 * np.cos(angle)), int(pos[1] + car_length / 2 * np.sin(angle)))
    pygame.draw.line(screen, BLACK, pos, front_center, 2)


def find_start_position(race_track, limits):
    """Find a valid starting position on the track"""
    # Use the beginning of the track as the starting point
    start_idx = 0
    start_pos = (race_track[0][start_idx], race_track[1][start_idx])

    # Calculate the track direction at the start position to set the car's initial angle
    # Using a few points ahead to get a better direction vector
    ahead_idx = min(start_idx + 10, len(race_track[0]) - 1)
    dx = race_track[0][ahead_idx] - race_track[0][start_idx]
    dy = race_track[1][ahead_idx] - race_track[1][start_idx]

    # Calculate the angle from the direction vector
    start_angle = np.arctan2(dy, dx)

    return start_pos, start_angle


def main():
    # Generate the track
    race_track = generate_race_track()
    limits = generate_road_limits(race_track, width=4)

    # Find a valid starting position
    start_pos, start_angle = find_start_position(race_track, limits)

    # Create the car with the proper starting position
    car = FuzzyCar(2, 4, max_speed=15, start_pos=start_pos, start_angle=start_angle)
    car_path = []

    # Find the track's bounding box for scaling
    min_x = min(np.min(limits[:, 0]), np.min(limits[:, 2]))
    max_x = max(np.max(limits[:, 0]), np.max(limits[:, 2]))
    min_y = min(np.min(limits[:, 1]), np.min(limits[:, 3]))
    max_y = max(np.max(limits[:, 1]), np.max(limits[:, 3]))

    # Add some margin
    margin = 50
    min_x -= margin
    min_y -= margin
    max_x += margin
    max_y += margin

    # Calculate scaling factor
    world_width = max_x - min_x
    world_height = max_y - min_y
    scale_x = WIDTH / world_width
    scale_y = HEIGHT / world_height
    scale = min(scale_x, scale_y)

    # Game loop variables
    clock = pygame.time.Clock()
    running = True
    paused = False
    game_over = False
    frame_counter = 0
    start_time = time.time()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Reset simulation with proper starting position
                    car = FuzzyCar(2, 4, max_speed=15, start_pos=start_pos, start_angle=start_angle)
                    car_path = []
                    game_over = False
                elif event.key == pygame.K_ESCAPE:
                    running = False

        if not paused and not game_over:
            # Update car position
            try:
                new_pos = car.update_position(limits, race_track)
                car_path.append(new_pos)
                # Keep only the last 100 positions for trail
                if len(car_path) > 100:
                    car_path = car_path[-100:]
            except Exception as e:
                if str(e) == "Fora da pista":
                    game_over = True
                    print("Car went off track!")
                else:
                    print(f"Error: {e}")
                    raise e

        # Clear the screen
        screen.fill(WHITE)

        # Draw track
        for i in range(len(limits) - 1):
            # Draw inner boundary
            p1 = world_to_screen((limits[i, 0], limits[i, 1]), min_x, min_y, scale)
            p2 = world_to_screen((limits[i + 1, 0], limits[i + 1, 1]), min_x, min_y, scale)
            pygame.draw.line(screen, GREEN, p1, p2, 2)

            # Draw outer boundary
            p1 = world_to_screen((limits[i, 2], limits[i, 3]), min_x, min_y, scale)
            p2 = world_to_screen((limits[i + 1, 2], limits[i + 1, 3]), min_x, min_y, scale)
            pygame.draw.line(screen, GREEN, p1, p2, 2)

        # Draw track center line
        for i in range(0, len(race_track[0]) - 1, 10):  # Draw every 10th point to optimize
            p1 = world_to_screen((race_track[0][i], race_track[1][i]), min_x, min_y, scale)
            p2 = world_to_screen((race_track[0][i + 1], race_track[1][i + 1]), min_x, min_y, scale)
            pygame.draw.line(screen, BLUE, p1, p2, 1)

        # Draw starting position for reference
        start_screen_pos = world_to_screen(start_pos, min_x, min_y, scale)
        pygame.draw.circle(screen, (255, 165, 0), start_screen_pos, 8)  # Orange circle

        # Draw car's path
        if len(car_path) > 1:
            for i in range(len(car_path) - 1):
                p1 = world_to_screen(car_path[i], min_x, min_y, scale)
                p2 = world_to_screen(car_path[i + 1], min_x, min_y, scale)
                pygame.draw.line(screen, RED, p1, p2, 2)

        # Draw the car
        if car_path:
            car_screen_pos = world_to_screen(car.current_pos, min_x, min_y, scale)
            draw_car(screen, car_screen_pos, car.curr_angle, scale)

        # Draw information panel
        info_panel = pygame.Surface((300, 150))
        info_panel.fill(GREY)
        info_panel.set_alpha(200)
        screen.blit(info_panel, (10, 10))

        # Show information
        speed_text = font.render(f"Speed: {car.curr_speed:.2f}", True, BLACK)
        screen.blit(speed_text, (20, 20))

        angle_text = font.render(f"Angle: {car.curr_angle * 180 / np.pi:.2f}°", True, BLACK)
        screen.blit(angle_text, (20, 45))

        pos_text = font.render(f"Position: ({car.current_pos[0]:.0f}, {car.current_pos[1]:.0f})", True, BLACK)
        screen.blit(pos_text, (20, 70))

        lap_text = font.render(f"Laps: {car.laps}", True, BLACK)
        screen.blit(lap_text, (20, 95))

        fps_text = font.render(f"FPS: {clock.get_fps():.1f}", True, BLACK)
        screen.blit(fps_text, (20, 120))

        # Show game status messages
        if paused:
            pause_text = font.render("PAUSED (Press SPACE to resume)", True, BLACK)
            screen.blit(pause_text, (WIDTH // 2 - 150, 20))

        if game_over:
            over_text = font.render("OFF TRACK! Press R to reset", True, RED)
            screen.blit(over_text, (WIDTH // 2 - 150, 60))

        # Controls info
        controls_text = font.render("Controls: SPACE - Pause/Resume, R - Reset, ESC - Quit", True, BLACK)
        screen.blit(controls_text, (WIDTH // 2 - 220, HEIGHT - 30))

        # Update the display
        pygame.display.flip()

        # Cap the framerate
        clock.tick(60)

        # Count frames for performance monitoring
        frame_counter += 1
        if frame_counter % 100 == 0:
            elapsed = time.time() - start_time
            print(f"Average FPS: {frame_counter / elapsed:.1f}")

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
