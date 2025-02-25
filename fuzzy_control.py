import pygame
import math

# Inicializa o pygame
pygame.init()

# Configurações da tela
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Slot Car Racing")

# Cores
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

# Pista (elipse com trechos retos)
track_center = (WIDTH // 2, HEIGHT // 2)
track_radius_x = 300
track_radius_y = 200
straight_length = 200

# Configuração do carro
angle = 0
speed = 0
max_speed = 5
curve_speed_threshold = 3.5  # Velocidade máxima permitida nas curvas
running = True
flying_off = False
fly_off_x, fly_off_y = 0, 0


def get_position(angle):
    """Calcula a posição do carro na pista considerando trechos retos."""
    adjusted_angle = angle % 360
    if 0 <= adjusted_angle < 45 or 135 <= adjusted_angle < 225 or 315 <= adjusted_angle < 360:
        x = track_center[0] + track_radius_x * math.cos(math.radians(angle))
        y = track_center[1] + track_radius_y * math.sin(math.radians(angle))
    else:
        x = track_center[0] + (track_radius_x if math.cos(math.radians(angle)) > 0 else -track_radius_x)
        y = track_center[1] + straight_length * math.sin(math.radians(angle))
    return x, y


def is_curve(angle):
    """Determina se o carro está em uma curva."""
    return (45 < angle % 360 < 135) or (225 < angle % 360 < 315)


def display_game_over():
    """Exibe mensagem de Game Over."""
    font = pygame.font.Font(None, 74)
    text = font.render("Game Over!", True, RED)
    screen.blit(text, (WIDTH // 2 - 100, HEIGHT // 2 - 50))
    pygame.display.flip()
    pygame.time.delay(2000)


def main():
    global angle, speed, running, flying_off, fly_off_x, fly_off_y
    clock = pygame.time.Clock()

    while running:
        screen.fill(WHITE)
        pygame.draw.ellipse(
            screen,
            BLACK,
            (
                track_center[0] - track_radius_x,
                track_center[1] - track_radius_y,
                track_radius_x * 2,
                track_radius_y * 2,
            ),
            5,
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            speed = min(speed + 0.1, max_speed)
        elif keys[pygame.K_DOWN]:
            speed = max(speed - 0.1, 0)

        if flying_off:
            fly_off_x += speed * 2
            fly_off_y -= speed * 2
            pygame.draw.rect(screen, RED, (fly_off_x, fly_off_y, 40, 20))
            if fly_off_x > WIDTH or fly_off_y < 0:
                display_game_over()
                running = False
        else:
            # Atualizar posição
            angle += speed
            x, y = get_position(angle)

            # Verifica se o carro deve sair da pista
            if is_curve(angle) and speed > curve_speed_threshold:
                flying_off = True
                fly_off_x, fly_off_y = x, y
                continue

            # Desenhar o carro como um retângulo
            car_width, car_height = 40, 20
            car_rect = pygame.Rect(x - car_width // 2, y - car_height // 2, car_width, car_height)
            pygame.draw.rect(screen, RED, car_rect)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


main()

# import numpy as np
# import pygame


# class Car:
#     def __init__(self, mass, friction, start_pos):
#         self.mass = mass
#         self.friction = friction
#         self.position = np.array(start_pos, dtype=float)
#         self.velocity = np.array([0.0, 0.0])
#         self.acceleration = np.array([0.0, 0.0])

#     def apply_acceleration(self, accel):
#         self.acceleration = np.array(accel)
#         self.velocity += self.acceleration

#         # Compute lateral velocity component
#         if np.linalg.norm(self.velocity) > 0:
#             tangent = self.velocity / np.linalg.norm(self.velocity)
#             lateral_velocity = self.velocity - np.dot(self.velocity, tangent) * tangent
#             self.velocity -= lateral_velocity * self.friction  # Applying friction only to lateral motion

#         self.position += self.velocity

#     def get_lateral_acceleration(self):
#         return np.linalg.norm(np.cross(self.velocity, self.acceleration)) / (np.linalg.norm(self.velocity) + 1e-5)


# class Circuit:
#     def __init__(self, waypoints, track_width):
#         self.waypoints = np.array(waypoints)
#         self.track_width = track_width

#     def is_on_track(self, position):
#         distances = np.linalg.norm(self.waypoints - position, axis=1)
#         return np.min(distances) < self.track_width / 2

#     def get_tangent_vector(self, position):
#         closest_index = np.argmin(np.linalg.norm(self.waypoints - position, axis=1))
#         next_index = (closest_index + 1) % len(self.waypoints)
#         tangent = self.waypoints[next_index] - self.waypoints[closest_index]
#         return tangent / np.linalg.norm(tangent)


# class Simulation:
#     def __init__(self, car, circuit, dt=0.1):
#         self.car = car
#         self.circuit = circuit
#         self.dt = dt
#         self.running = True
#         self.acceleration_factor = 0.5

#         pygame.init()
#         self.screen = pygame.display.set_mode((800, 600))
#         self.clock = pygame.time.Clock()

#     def run(self):
#         while self.running:
#             self.handle_events()
#             self.update()
#             self.render()
#             self.clock.tick(10)

#     def handle_events(self):
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 self.running = False

#         keys = pygame.key.get_pressed()
#         if keys[pygame.K_UP]:
#             self.acceleration_factor += 0.1
#         if keys[pygame.K_DOWN]:
#             self.acceleration_factor -= 0.1
#         self.acceleration_factor = max(0, self.acceleration_factor)

#     def update(self):
#         tangent = self.circuit.get_tangent_vector(self.car.position)
#         applied_accel = self.acceleration_factor * tangent
#         self.car.apply_acceleration(applied_accel)

#         if not self.circuit.is_on_track(self.car.position):
#             print("Car went off track!")
#             self.running = False

#     def render(self):
#         self.screen.fill((0, 0, 0))

#         for waypoint in self.circuit.waypoints:
#             pygame.draw.circle(
#                 self.screen, (255, 255, 255), (int(waypoint[0] * 10 + 400), int(waypoint[1] * 10 + 300)), 2
#             )

#         pygame.draw.circle(
#             self.screen, (255, 0, 0), (int(self.car.position[0] * 10 + 400), int(self.car.position[1] * 10 + 300)), 5
#         )
#         pygame.display.flip()


# # Mosport Road Course Waypoints
# waypoints = [
#     (0, 0),
#     (10, 5),
#     (20, 10),
#     (30, 8),
#     (40, 5),
#     (50, 0),
#     (45, -5),
#     (35, -10),
#     (25, -15),
#     (15, -20),
#     (5, -25),
#     (-5, -20),
#     (-15, -15),
#     (-25, -10),
#     (-35, -5),
#     (-40, 0),
#     (-30, 5),
#     (-20, 10),
#     (-10, 8),
#     (0, 0),
# ]

# circuit = Circuit(waypoints=waypoints, track_width=10)
# car = Car(mass=2000, friction=0.05, start_pos=waypoints[0])

# sim = Simulation(car, circuit)
# sim.run()
# pygame.quit()
