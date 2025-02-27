import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from skfuzzy import control as ctrl
import numpy as np


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
        * 1000
    )  # Transpose for compatibility

    # Generate a smooth spline curve through control points
    tck, u = splprep(control_points, s=0, per=True)  # Closed curve
    u_fine = np.linspace(0, 1, 5000)  # Increased number of points for smoothness
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


# Gerar os pontos suavizados da pista
race_track = generate_race_track()

# Gerar os limites da estrada com largura constante de 20 metros
limits = generate_road_limits(race_track)


class FuzzyCar:
    def __init__(self, width, length, max_speed=10):
        self.width = width
        self.length = length
        self.max_speed = max_speed
        self.curr_speed = 0
        self.curr_angle = -np.pi / 2
        self.current_pos = (0, 0)
        self.state = pd.DataFrame(
            [[0, 0, 0, 0, *self.velocity()]],
            columns=["Proximity Diff.", "Delta Diff.", "Delta Angle", "Acceleration", "Speed", "Angle"],
        )

        proximity_diff = ctrl.Antecedent(np.arange(-0.5, 0.51, 0.001), "proximity_diff")
        delta_diff = ctrl.Antecedent(np.arange(-0.05, 0.051, 0.0001), "delta_diff")

        acceleration = ctrl.Consequent(np.arange(-0.1, 0.1, 0.001), "acceleration")

        # Positivo: margem direita mais próxima
        proximity_diff.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])
        delta_diff.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])

        # Positivo: sentido anti-horário
        max_radian = 15 * np.pi / 180  # 15 degrees in radians

        # Create the antecedent
        delta_angle = ctrl.Consequent(np.arange(-max_radian, max_radian, 0.001), "angle")

        delta_angle.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])
        acceleration.automf(names=["NG", "NM", "NP", "ZE", "PP", "PM", "PG"])

        rules = [
            # Angulação
            # ctrl.Rule(proximity_diff["NG"], delta_angle["NG"]),
            # ctrl.Rule(proximity_diff["NM"], delta_angle["NM"]),
            # ctrl.Rule(proximity_diff["NP"], delta_angle["NP"]),
            # ctrl.Rule(proximity_diff["ZE"], delta_angle["ZE"]),
            # ctrl.Rule(proximity_diff["PP"], delta_angle["PP"]),
            # ctrl.Rule(proximity_diff["PM"], delta_angle["PM"]),
            # ctrl.Rule(proximity_diff["PG"], delta_angle["PG"]),
            # Aceleração
            # ctrl.Rule(proximity_diff["NG"], acceleration["NM"]),
            # ctrl.Rule(proximity_diff["NM"], acceleration["NP"]),
            # ctrl.Rule(proximity_diff["NP"], acceleration["PP"]),
            # ctrl.Rule(proximity_diff["ZE"], acceleration["PM"]),
            # ctrl.Rule(proximity_diff["PP"], acceleration["PP"]),
            # ctrl.Rule(proximity_diff["PM"], acceleration["NP"]),
            # ctrl.Rule(proximity_diff["PG"], acceleration["NM"]),
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
        # self.fuzzy_sim.input["proximity_diff"] = 0.0
        # self.fuzzy_sim.input["delta_diff"] = 0.0
        # self.fuzzy_sim.compute()
        # acceleration.view(sim=self.fuzzy_sim)

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

    def update_position(self, limits):
        proximity_diff = self.nearest_limit_points_diff(limits)
        delta_diff = proximity_diff - self.state.values[-1][0]

        self.fuzzy_sim.input["proximity_diff"] = proximity_diff
        self.fuzzy_sim.input["delta_diff"] = delta_diff

        self.fuzzy_sim.compute()
        angle_delta = self.fuzzy_sim.output["angle"]
        accel = self.fuzzy_sim.output["acceleration"] * 2

        self.update_state(proximity_diff, delta_diff, angle_delta, accel)

        self.curr_speed += accel
        self.curr_speed = np.clip(self.curr_speed, 0, self.max_speed)

        self.curr_angle += angle_delta

        self.current_pos = self.current_pos + self.curr_speed * np.array(
            [np.sin(self.curr_angle), np.cos(self.curr_angle)]
        )

        return self.current_pos


car = FuzzyCar(2, 4)
car_path = []

for _ in range(len(race_track[0])):
    try:
        pos = car.update_position(limits)
    except Exception as e:
        if str(e) == "Fora da pista":
            print("FORA DA PISTA")
            break
        else:
            print(car.state)
            raise e

    car_path.append(pos)

car_path = np.array(car_path)


def calculate_plot_limits(car_path, margin=0.1):
    # Ensure car_path is a numpy array
    car_path = np.array(car_path)

    # Extract x and y coordinates
    x = car_path[:, 0]
    y = car_path[:, 1]

    # Calculate bounds for zooming
    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)

    # Add margin
    margin_x = margin * (x_max - x_min)
    margin_y = margin * (y_max - y_min)

    # Return the calculated limits
    return (x_min - margin_x, x_max + margin_x), (y_min - margin_y, y_max + margin_y)


# # Plotar a pista e os limites da estrada
xlim, ylim = calculate_plot_limits(car_path, margin=1)
plt.figure(figsize=(10, 6))
plt.scatter(race_track[0], race_track[1], label="Centro da Pista", color="blue", linestyle="--", s=0.5)
plt.plot(car_path[:, 0], car_path[:, 1], label="Caminho do carro", color="red")
# plt.plot(limits[:, 0], limits[:, 1], label="Limite Interno", color="green")
# plt.plot(limits[:, 2], limits[:, 3], label="Limite Externo", color="green")
plt.title("Pista de Corrida")
plt.xlabel("X (metros)")
plt.ylabel("Y (metros)")
plt.axis("equal")
plt.legend()

plt.xlim(xlim)
plt.ylim(ylim)

# Set equal aspect ratio
plt.show()
car.state
