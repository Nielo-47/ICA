import pygame
import math
import numpy as np
from dataclasses import dataclass
import time
from typing import List, Tuple


@dataclass
class Car:
    mass: float  # kg
    friction_coefficient: float
    position: pygame.Vector2
    velocity: pygame.Vector2
    heading: float  # radians
    lateral_acceleration: float = 0
    length: float = 4.0  # meters
    width: float = 2.0  # meters


class CircuitSimulation:
    def __init__(self, screen_width=1200, screen_height=800, pixels_per_meter=10):
        pygame.init()
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        self.clock = pygame.time.Clock()
        self.pixels_per_meter = pixels_per_meter

        # Generate track waypoints
        self.waypoints = self.generate_track()
        self.current_waypoint = 0

        # Create car instance
        self.car = Car(
            mass=1500,  # kg
            friction_coefficient=0.7,
            position=pygame.Vector2(self.waypoints[0]),
            velocity=pygame.Vector2(0, 0),
            heading=0,
        )

        self.running = True
        self.last_time = time.time()
        self.user_acceleration = 0  # m/s²
        self.road_width = self.car.width * 2  # Road width is twice the car's width
        self.crashed = False  # Track if the car has gone off-road

    def meters_to_pixels(self, meters):
        return int(meters * self.pixels_per_meter)

    def generate_track(self) -> List[pygame.Vector2]:
        """Generate a simple oval track"""
        waypoints = []
        center_x, center_y = self.screen.get_width() // 2, self.screen.get_height() // 2
        radius_x, radius_y = 300, 200  # Semi-major and semi-minor axes

        # Generate more waypoints for smoother navigation
        for angle in np.linspace(0, 2 * math.pi, 100):
            x = center_x + radius_x * math.cos(angle)
            y = center_y + radius_y * math.sin(angle)
            waypoints.append(pygame.Vector2(x, y))

        return waypoints

    def calculate_steering(self) -> float:
        """Calculate required steering angle to next waypoint"""
        target = self.waypoints[self.current_waypoint]
        to_target = target - self.car.position

        # If close enough to current waypoint, move to next one
        if to_target.length() < 20:
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
            target = self.waypoints[self.current_waypoint]
            to_target = target - self.car.position

        # Calculate angle to target
        target_angle = math.atan2(to_target.y, to_target.x)
        angle_diff = (target_angle - self.car.heading) % (2 * math.pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi

        return angle_diff * 2.0

    def calculate_forces(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()

        # Calculate steering
        steering = self.calculate_steering()
        self.car.heading += steering * dt

        # Convert heading to direction vector
        direction = pygame.Vector2(math.cos(self.car.heading), math.sin(self.car.heading))

        # Apply acceleration in car's forward direction
        if self.user_acceleration != 0:  # Only apply friction when accelerating or braking
            acceleration_vector = direction * self.user_acceleration

            # Calculate velocity
            self.car.velocity += acceleration_vector * dt

            # Apply friction only when accelerating/braking
            friction_force = -self.car.velocity * self.car.friction_coefficient
            self.car.velocity += friction_force * dt

        # Update position using current velocity
        self.car.position += self.car.velocity * dt

        # Calculate lateral acceleration
        velocity_magnitude = self.car.velocity.length()
        if velocity_magnitude > 0:
            velocity_direction = self.car.velocity / velocity_magnitude
            angle_diff = math.acos(direction.dot(velocity_direction))
            self.car.lateral_acceleration = velocity_magnitude * math.sin(angle_diff)
        else:
            self.car.lateral_acceleration = 0

    def is_car_on_road(self) -> bool:
        """Check if the car is within the road boundaries"""
        # Find the nearest segment of the track
        min_distance = float("inf")
        nearest_point = None
        for i in range(len(self.waypoints)):
            start = self.waypoints[i]
            end = self.waypoints[(i + 1) % len(self.waypoints)]

            # Vector from start to end of the segment
            segment = end - start
            segment_length = segment.length()
            segment_direction = segment / segment_length

            # Vector from start to car position
            to_car = self.car.position - start

            # Project to_car onto the segment
            projection = to_car.dot(segment_direction)

            # Clamp the projection to the segment
            projection = max(0, min(projection, segment_length))

            # Find the closest point on the segment
            closest_point = start + segment_direction * projection

            # Calculate distance from the car to the closest point
            distance = (self.car.position - closest_point).length()

            # Update the nearest point
            if distance < min_distance:
                min_distance = distance
                nearest_point = closest_point

        # Check if the car is within the road width
        return min_distance <= self.road_width

    def draw_road(self):
        """Render the road by drawing the boundaries and filling the area between them"""
        if len(self.waypoints) < 2:
            return

        # Draw the road as a filled polygon
        inner_boundary = []
        outer_boundary = []

        for i in range(len(self.waypoints)):
            start = self.waypoints[i]
            end = self.waypoints[(i + 1) % len(self.waypoints)]

            # Calculate the direction of the segment
            segment = end - start
            segment_length = segment.length()
            segment_direction = segment / segment_length

            # Calculate the perpendicular direction
            perpendicular = pygame.Vector2(-segment_direction.y, segment_direction.x)

            # Calculate the inner and outer boundary points
            inner_point = start + perpendicular * (-self.road_width / 2)
            outer_point = start + perpendicular * (self.road_width / 2)

            inner_boundary.append(inner_point)
            outer_boundary.append(outer_point)

        # Combine the boundaries to form a polygon
        road_polygon = inner_boundary + outer_boundary[::-1]

        # Draw the filled polygon
        pygame.draw.polygon(self.screen, (200, 200, 200), road_polygon)

        # Draw the road boundaries
        pygame.draw.lines(self.screen, (0, 0, 0), True, inner_boundary, 2)
        pygame.draw.lines(self.screen, (0, 0, 0), True, outer_boundary, 2)

    def draw(self):
        self.screen.fill((255, 255, 255))

        # Draw the road
        self.draw_road()

        # Draw waypoints
        for i, point in enumerate(self.waypoints):
            color = (255, 0, 0) if i == self.current_waypoint else (128, 128, 128)
            pygame.draw.circle(self.screen, color, point, 3)

        # Draw car
        car_surface = pygame.Surface(
            (self.meters_to_pixels(self.car.length), self.meters_to_pixels(self.car.width)), pygame.SRCALPHA
        )

        pygame.draw.rect(car_surface, (0, 0, 255), car_surface.get_rect())

        # Rotate car surface
        rotated_surface = pygame.transform.rotate(car_surface, -math.degrees(self.car.heading))

        # Draw car at its position
        car_rect = rotated_surface.get_rect(center=self.car.position)
        self.screen.blit(rotated_surface, car_rect)

        # Draw telemetry
        font = pygame.font.Font(None, 36)
        texts = [
            f"Lateral Acceleration: {self.car.lateral_acceleration:.2f} m/s²",
            f"Speed: {self.car.velocity.length():.2f} m/s",
            f"Target Acceleration: {self.user_acceleration:.2f} m/s²",
        ]

        for i, text in enumerate(texts):
            surface = font.render(text, True, (0, 0, 0))
            self.screen.blit(surface, (10, 10 + i * 30))

        # Show crash warning if off-road
        if self.crashed:
            warning_text = font.render("CRASHED! Press SPACE to restart", True, (255, 0, 0))
            self.screen.blit(warning_text, (self.screen.get_width() // 2 - 200, self.screen.get_height() // 2))

        pygame.display.flip()

    def restart_simulation(self):
        """Reset the simulation to its initial state"""
        self.car.position = pygame.Vector2(self.waypoints[0])
        self.car.velocity = pygame.Vector2(0, 0)
        self.car.heading = 0
        self.user_acceleration = 0
        self.crashed = False

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    # Increase/decrease acceleration on key press
                    if event.key == pygame.K_UP:
                        self.user_acceleration = min(self.user_acceleration + 5, 30)  # m/s²
                    elif event.key == pygame.K_DOWN:
                        self.user_acceleration = max(self.user_acceleration - 5, -10)  # m/s²
                    elif event.key == pygame.K_SPACE and self.crashed:
                        self.restart_simulation()

            # Check if the car is off-road
            if not self.crashed and not self.is_car_on_road():
                self.crashed = True
                self.user_acceleration = 0  # Stop the car

            # Calculate physics only if not crashed
            if not self.crashed:
                self.calculate_forces()

            # Draw everything
            self.draw()

            # Maintain 60 FPS
            self.clock.tick(60)

        pygame.quit()


if __name__ == "__main__":
    simulation = CircuitSimulation()
    simulation.run()
