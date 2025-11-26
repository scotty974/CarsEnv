import matplotlib.pyplot as plt
import numpy as np
from pandas.core.arrays.datetimelike import F
from utils.calculs import (
    calcul_acceleration,
    calcul_aero_drag,
    calcul_consommation,
    calcul_engine_power,
    calcul_engine_speed,
    calcul_pente,
    calcul_position,
    calcul_roll,
    calcul_vitesse,
    make_interpolated_function,
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
        self.F_max = 8000
        self.v_max = 66.66
        self.P_max = 331000  # en W

        self.data = []
        self.transmission_ratio = transmission_ratio
        self.wheel_radius = wheel_radius
        self.bsfc = 0.280  # kg/kwh

    def get_state_min(self):
        return self.state_min

    def reset_state(self):
        self.state_min = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.data = []

    def step(self, actions):
        v_current, x_current, _, _, consommation_totale = self.state_min

        # Calcul des résistances
        f_aero = calcul_aero_drag(
            self.air_density,
            self.drag_coefficient,
            self.surface_frontal_area,
            v_current,
        )
        f_roll = calcul_roll(self.roll_coefficient, self.mass, self.gravity)
        f_pente = calcul_pente(self.mass, self.gravity, self.angle)

        # Régime moteur
        omega_engine = calcul_engine_speed(
            abs(v_current),  # Utiliser valeur absolue pour éviter régimes négatifs
            self.wheel_radius,
            self.transmission_ratio,
        )
        rpm = omega_engine * 60 / (2 * np.pi)
        c = make_interpolated_function(rpm)  # Couple en Nm

        if actions == 1:
            # Couple effectif limité par la puissance max
            if omega_engine > 0:
                T_limite = self.P_max / omega_engine
                T_effectif = min(c, T_limite)
            else:
                T_effectif = 0  # Au démarrage (omega=0), on prend le couple de la map

            # ⚠️ CORRECTION MAJEURE : Force aux roues
            # Le couple moteur est multiplié par le rapport de transmission
            # puis divisé par le rayon de roue pour obtenir la force
            F_moteur = (T_effectif * self.transmission_ratio) / self.wheel_radius

            # Puissance moteur
            power = calcul_engine_power(T_effectif, omega_engine)

        else:
            # Freinage
            if v_current > 0.1:
                F_moteur = -self.F_max
            elif v_current < -0.1:
                F_moteur = self.F_max
            else:
                F_moteur = 0

            power = 0  # Pas de puissance moteur en freinage

        # ⚠️ CORRECTION : F_moteur EST DÉJÀ la force aux roues
        F_roue = F_moteur

        # Force résultante (SANS signe négatif devant f_pente si votre fonction retourne déjà le bon signe)
        array = np.array([F_roue, -f_aero, -f_roll, -f_pente])
        f_result = result_force(array)

        # Cinématique
        a = calcul_acceleration(self.mass, f_result)
        v = calcul_vitesse(v_current, a, self.dt)

        # Limitation de vitesse
        v = max(-self.v_max, min(self.v_max, v))
        if actions == 0:  # Freinage
            if (v_current > 0 and v < 0) or (v_current < 0 and v > 0):
                v = 0

        x = calcul_position(x_current, v, self.dt)

        # ⚠️ CORRECTION CONSOMMATION : Toujours positive ou nulle
        if actions == 1 and power > 0:
            consommation_instantanee = calcul_consommation(power, self.bsfc)
            consommation_totale += consommation_instantanee * (self.dt / 3600)
        # Sinon on ne consomme rien (freinage ou moteur arrêté)

        # Mise à jour
        self.state_min = [v, x, a, power, consommation_totale]
        self.data.append(self.state_min.copy())

        print(
            f"{'-' * 40}\n"
            f"Vitesse       : {v:.2f} m/s ({v * 3.6:.2f} km/h)\n"
            f"Position      : {x:.2f} m\n"
            f"Accélération  : {a:.2f} m/s²\n"
            f"Puissance mot : {power:.2f} W\n"
            f"Consommation  : {consommation_totale:.4f} kg\n"
            f"--- Debug ---\n"
            f"F_moteur      : {F_moteur:.2f} N\n"
            f"F_aero        : {f_aero:.2f} N\n"
            f"F_roll        : {f_roll:.2f} N\n"
            f"F_pente       : {f_pente:.2f} N\n"
            f"F_resultante  : {f_result:.2f} N\n"
            f"RPM           : {rpm:.0f}\n"
            f"Couple        : {c:.2f} Nm\n"
            f"{'-' * 40}\n"
        )

    def plot_data(self):
        if not self.data:
            print("Aucune donnée à afficher")
            return

        # Préparer les données
        temps = np.arange(len(self.data)) * self.dt

        # Extraire toutes les données
        vitesses = [state[0] for state in self.data]
        positions = [state[1] for state in self.data]
        accelerations = [state[2] for state in self.data]
        puissances = [state[3] for state in self.data]
        consommations = [state[4] for state in self.data]

        # Calculer les données supplémentaires
        vitesses_kmh = [v * 3.6 for v in vitesses]

        # RPM et couple
        rpms = []
        couples = []
        forces_motrices = []

        for v in vitesses:
            omega = calcul_engine_speed(
                abs(v), self.wheel_radius, self.transmission_ratio
            )
            rpm = omega * 60 / (2 * np.pi)
            c = make_interpolated_function(rpm)
            F_moteur = (c * self.transmission_ratio) / self.wheel_radius

            rpms.append(rpm)
            couples.append(c)
            forces_motrices.append(F_moteur)

        # Consommation instantanée (L/100km)
        conso_instantanee = []
        for i, v in enumerate(vitesses):
            if abs(v) > 0.1 and puissances[i] > 0:
                # Consommation en kg/s
                conso_kg_s = puissances[i] * self.bsfc / 3600
                # Convertir en L/100km (densité essence ~0.75 kg/L)
                conso_L_100km = (conso_kg_s / 0.75) * (3600 / abs(v)) * 100000
                conso_instantanee.append(
                    min(conso_L_100km, 50)
                )  # Limiter à 50 L/100km pour lisibilité
            else:
                conso_instantanee.append(0)

        # Créer la figure avec 8 subplots (2 colonnes)
        fig = plt.figure(figsize=(16, 12))

        # Subplot 1: Vitesse (km/h)
        ax1 = plt.subplot(4, 2, 1)
        ax1.plot(temps, vitesses_kmh, "b-", linewidth=2)
        ax1.set_ylabel("Vitesse (km/h)", fontsize=11, fontweight="bold")
        ax1.grid(True, alpha=0.3)
        ax1.set_title("Profil de vitesse", fontsize=12, fontweight="bold")

        # Subplot 2: Position
        ax2 = plt.subplot(4, 2, 2)
        ax2.plot(temps, [p / 1000 for p in positions], "g-", linewidth=2)
        ax2.set_ylabel("Distance (km)", fontsize=11, fontweight="bold")
        ax2.grid(True, alpha=0.3)
        ax2.set_title("Distance parcourue", fontsize=12, fontweight="bold")

        # Subplot 3: Accélération
        ax3 = plt.subplot(4, 2, 3)
        ax3.plot(temps, accelerations, "r-", linewidth=2)
        ax3.axhline(y=0, color="k", linestyle="--", alpha=0.3)
        ax3.set_ylabel("Accélération (m/s²)", fontsize=11, fontweight="bold")
        ax3.grid(True, alpha=0.3)
        ax3.set_title("Accélération", fontsize=12, fontweight="bold")

        # Subplot 4: Régime moteur (RPM)
        ax4 = plt.subplot(4, 2, 4)
        ax4.plot(temps, rpms, "purple", linewidth=2)
        ax4.set_ylabel("Régime (RPM)", fontsize=11, fontweight="bold")
        ax4.grid(True, alpha=0.3)
        ax4.set_title("Régime moteur", fontsize=12, fontweight="bold")

        # Subplot 5: Couple moteur
        ax5 = plt.subplot(4, 2, 5)
        ax5.plot(temps, couples, "orange", linewidth=2)
        ax5.set_ylabel("Couple (Nm)", fontsize=11, fontweight="bold")
        ax5.grid(True, alpha=0.3)
        ax5.set_title("Couple moteur", fontsize=12, fontweight="bold")

        # Subplot 6: Courbe Couple-Régime (comme sur une fiche technique)
        ax6 = plt.subplot(4, 2, 6)
        ax6.plot(
            rpms,
            couples,
            "o-",
            color="darkorange",
            linewidth=2,
            markersize=3,
            alpha=0.6,
        )
        ax6.set_xlabel("Régime (RPM)", fontsize=11, fontweight="bold")
        ax6.set_ylabel("Couple (Nm)", fontsize=11, fontweight="bold")
        ax6.grid(True, alpha=0.3)
        ax6.set_title("Courbe Couple/Régime", fontsize=12, fontweight="bold")

        # Subplot 7: Puissance moteur
        ax7 = plt.subplot(4, 2, 7)
        ax7.plot(temps, [p / 1000 for p in puissances], "cyan", linewidth=2)
        ax7.set_xlabel("Temps (s)", fontsize=11, fontweight="bold")
        ax7.set_ylabel("Puissance (kW)", fontsize=11, fontweight="bold")
        ax7.grid(True, alpha=0.3)
        ax7.set_title("Puissance moteur", fontsize=12, fontweight="bold")

        # Subplot 8: Consommation instantanée
        ax8 = plt.subplot(4, 2, 8)
        ax8.plot(temps, conso_instantanee, "brown", linewidth=2)
        ax8.set_xlabel("Temps (s)", fontsize=11, fontweight="bold")
        ax8.set_ylabel("Consommation (L/100km)", fontsize=11, fontweight="bold")
        ax8.grid(True, alpha=0.3)
        ax8.set_title("Consommation instantanée", fontsize=12, fontweight="bold")

        # Ajuster l'espacement
        plt.tight_layout()

        # Ajouter un titre global
        fig.suptitle(
            "Simulation de conduite - Analyse complète",
            fontsize=16,
            fontweight="bold",
            y=0.998,
        )
        plt.subplots_adjust(top=0.96)

        plt.show()

        # Afficher aussi la courbe couple/régime théorique
        self.plot_torque_curve()

    def plot_torque_curve(self):
        """Affiche la courbe de couple théorique du moteur"""
        from utils.calculs import torque_map_rpm

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

        # Extraire les données de la map
        rpms_map = [p for p, t in torque_map_rpm]
        torques_map = [t for p, t in torque_map_rpm]

        # Générer une courbe interpolée
        rpms_interp = np.linspace(min(rpms_map), max(rpms_map), 200)
        torques_interp = [make_interpolated_function(rpm) for rpm in rpms_interp]
        puissances_interp = [
            t * (rpm * 2 * np.pi / 60) / 1000
            for t, rpm in zip(torques_interp, rpms_interp)
        ]

        # Graphique 1: Couple
        ax1.plot(rpms_interp, torques_interp, "b-", linewidth=2, label="Couple")
        ax1.plot(rpms_map, torques_map, "ro", markersize=8, label="Points de mesure")
        ax1.set_xlabel("Régime moteur (RPM)", fontsize=12, fontweight="bold")
        ax1.set_ylabel("Couple (Nm)", fontsize=12, fontweight="bold", color="b")
        ax1.tick_params(axis="y", labelcolor="b")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="upper right")
        ax1.set_title("Courbe de couple moteur", fontsize=13, fontweight="bold")

        # Graphique 2: Puissance
        ax2.plot(rpms_interp, puissances_interp, "r-", linewidth=2, label="Puissance")
        ax2.axhline(
            y=self.P_max / 1000,
            color="orange",
            linestyle="--",
            label=f"P_max = {self.P_max / 1000:.0f} kW",
        )
        ax2.set_xlabel("Régime moteur (RPM)", fontsize=12, fontweight="bold")
        ax2.set_ylabel("Puissance (kW)", fontsize=12, fontweight="bold", color="r")
        ax2.tick_params(axis="y", labelcolor="r")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="upper left")
        ax2.set_title("Courbe de puissance moteur", fontsize=13, fontweight="bold")

        plt.tight_layout()
        plt.show()
