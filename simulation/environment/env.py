import matplotlib.pyplot as plt
import numpy as np
from utils.calculs import (
    calcul_acceleration,
    calcul_aero_drag,
    calcul_consommation,
    calcul_motor_power,
    calcul_pente,
    calcul_position,
    calcul_roll,
    calcul_vitesse,
    result_force,
)


class Environment:
    def __init__(
        self,
        drag_coefficient,
        air_density,
        surface_frontal_area,
        roll_coefficient,
        mass,
        angle,
        transmission_ratio,
        wheel_radius,
    ):
        self.gravity = 9.81
        # (vitesse, position, accélération, puissance moteur, consommation TOTALE)
        self.state_min = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.air_density = air_density
        self.surface_frontal_area = surface_frontal_area
        self.drag_coefficient = drag_coefficient
        self.roll_coefficient = roll_coefficient
        self.mass = mass
        self.angle = angle
        self.dt = 1.0

        # Paramètres moteur CORRIGÉS
        self.F_max = 400
        self.v_max = 47.2
        self.P_max = 60000  # en W

        self.data = []
        self.transmission_ratio = transmission_ratio
        self.wheel_radius = wheel_radius
        self.bsfc = 0.25  # kg/kwh

    def get_state_min(self):
        return self.state_min

    def reset_state(self):
        self.state_min = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.data = []

    def step(self, actions):
        v_current, x_current, _, _, consommation_totale = self.state_min

        # UTILISATION DE VOS FONCTIONS
        f_aero = calcul_aero_drag(
            self.air_density,
            self.drag_coefficient,
            self.surface_frontal_area,
            v_current,
        )

        f_roll = calcul_roll(self.roll_coefficient, self.mass, self.gravity)

        f_pente = calcul_pente(self.mass, self.gravity, self.angle)

        if actions == 1:
            # La puissance limite la force : P = F × v donc F = P / v
            F_moteur_demande = self.F_max
            # Éviter division par zéro + limiter par la puissance
            F_moteur = min(F_moteur_demande, self.P_max / max(abs(v_current), 1))
        else:
            # Freinage : force opposée au mouvement pour éviter la marche arrière
            if v_current > 0.1:  # Si on avance
                F_moteur = -self.F_max
            elif v_current < -0.1:  # Si on recule
                F_moteur = self.F_max
            else:  # Si quasi à l'arrêt
                F_moteur = 0

        F_roue = F_moteur * self.transmission_ratio / self.wheel_radius

        # Calcul force résultante avec vos fonctions
        array = np.array([F_roue, -f_aero, -f_roll, -f_pente])
        f_result = result_force(array)

        # Cinématique avec vos fonctions
        a = calcul_acceleration(self.mass, f_result)
        v = calcul_vitesse(v_current, a, self.dt)

        # LIMITATION DE VITESSE
        v = max(-self.v_max, min(self.v_max, v))
        if actions == 0:  # Freinage
            if (v_current > 0 and v < 0) or (v_current < 0 and v > 0):
                v = 0
        x = calcul_position(x_current, v, self.dt)

        # Puissance et consommation avec vos fonctions
        p_roue = calcul_motor_power(F_roue, abs(v))
        consommation_instantanee = calcul_consommation(p_roue, self.bsfc)

        # CONSOMMATION CUMULÉE (correction)
        consommation_totale += consommation_instantanee * (self.dt / 3600)

        # Mise à jour
        self.state_min = [v, x, a, p_roue, consommation_totale]
        self.data.append(self.state_min.copy())

        print(
            f"{'-' * 40}\n"
            f"Vitesse       : {v:.2f} m/s ({v * 3.6:.2f} km/h)\n"
            f"Position      : {x:.2f} m\n"
            f"Accélération  : {a:.2f} m/s²\n"
            f"Puissance mot : {p_roue:.2f} W\n"
            f"Consommation  : {consommation_totale:.4f} kg\n"
            f"{'-' * 40}\n"
        )

    def plot_data(self):
        if not self.data:
            print("Aucune donnée à afficher")
            return

        fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
        temps = np.arange(len(self.data)) * self.dt

        labels = [
            ("Position (m)", 1),
            ("Vitesse (m/s)", 0),
            ("Accélération (m/s²)", 2),
            ("Puissance moteur (W)", 3),
            ("Consommation totale (kg)", 4),
        ]

        for i, (label, idx) in enumerate(labels):
            data_serie = [state[idx] for state in self.data]
            axs[i].plot(temps, data_serie)
            axs[i].set_ylabel(label)
            axs[i].grid(True)

        axs[-1].set_xlabel("Temps (s)")
        plt.tight_layout()
        plt.show()
