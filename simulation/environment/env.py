import matplotlib.pyplot as plt
import numpy as np
from pandas.core.arrays.datetimelike import F
from utils.calculs import (
    calcul_acceleration,
    calcul_aero_drag,
    calcul_brake_force,
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
        self.brake_force_max = 0.0

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
                T_effectif = c

            F_roue = (T_effectif * self.transmission_ratio) / self.wheel_radius
            power = calcul_engine_power(T_effectif, omega_engine)

        else:
            F_frein_max = calcul_brake_force(0.8, self.mass, self.gravity)
            F_frein = F_frein_max * (abs(v_current) / self.v_max)

            # Freinage -> force opposée au mouvement
            if v_current > 0:
                F_roue = -F_frein
            elif v_current < 0:
                F_roue = F_frein
            else:
                F_roue = 0

            power = 0

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

        if actions == 1 and power > 0:
            consommation_instantanee = calcul_consommation(power, self.bsfc)
            consommation_totale += consommation_instantanee * (self.dt / 3600)

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
            f"F_roue      : {F_roue:.2f} N\n"
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

        # --- 1. Préparation des données ---
        temps = np.arange(len(self.data)) * self.dt

        # Extraction
        vitesses = np.array([state[0] for state in self.data])
        positions = np.array([state[1] for state in self.data])
        accelerations = np.array([state[2] for state in self.data])
        puissances = np.array([state[3] for state in self.data])
        conso_cumul_kg = np.array([state[4] for state in self.data])  # Conso cumulée

        # Conversions
        vitesses_kmh = vitesses * 3.6
        positions_km = positions / 1000
        puissances_kw = puissances / 1000

        # Recalcul RPM et Couple pour l'analyse
        rpms = []
        couples_utilises = []

        for v, p_kw in zip(vitesses, puissances_kw):
            omega = calcul_engine_speed(
                abs(v), self.wheel_radius, self.transmission_ratio
            )
            rpm = omega * 60 / (2 * np.pi)
            rpms.append(rpm)

            # Couple réel utilisé (déduit de la puissance: C = P / w)
            if omega > 0 and p_kw > 0:
                c_used = (p_kw * 1000) / omega
            else:
                c_used = 0
            couples_utilises.append(c_used)

        # Calcul Conso Instantanée (L/100km) avec gestion des erreurs
        conso_l_100 = []
        for i, v in enumerate(vitesses):
            # On évite la division par zéro et les valeurs aberrantes à très basse vitesse
            if abs(v) > 1.0 and puissances[i] > 0:
                # Conso (kg/s) = Power (kW) * BSFC (kg/kWh) / 3600
                dm_dt = (puissances[i] * self.bsfc) / 3600
                # L/100km = (L/s) / (km/s) * 100
                # Densité essence approx 0.75 kg/L
                vol_rate = dm_dt / 0.75
                km_s = abs(v) / 1000
                val = (vol_rate / km_s) * 100
                conso_l_100.append(
                    min(val, 60)
                )  # Cap à 60 L/100 pour lisibilité graphique
            else:
                conso_l_100.append(0)

        # --- Style global ---
        plt.style.use("ggplot")  # Style plus propre (ou 'seaborn-darkgrid')

        # ==========================================
        # FIGURE 1 : DYNAMIQUE VÉHICULE
        # ==========================================
        fig1, (ax1, ax2, ax3) = plt.subplots(
            3, 1, figsize=(10, 10), sharex=True, constrained_layout=True
        )
        fig1.suptitle("Dynamique du Véhicule", fontsize=16, fontweight="bold")

        # 1. Vitesse
        ax1.plot(temps, vitesses_kmh, color="tab:blue", linewidth=2)
        ax1.set_ylabel("Vitesse (km/h)", fontweight="bold")
        ax1.grid(True, alpha=0.5)
        ax1.fill_between(
            temps, vitesses_kmh, color="tab:blue", alpha=0.1
        )  # Effet de remplissage

        # 2. Accélération
        ax2.plot(temps, accelerations, color="tab:red", linewidth=1.5)
        ax2.set_ylabel("Accélération (m/s²)", fontweight="bold")
        ax2.axhline(0, color="black", linestyle="--", alpha=0.5)  # Ligne zéro
        ax2.grid(True, alpha=0.5)

        # 3. Position
        ax3.plot(temps, positions_km, color="tab:green", linewidth=2)
        ax3.set_ylabel("Distance (km)", fontweight="bold")
        ax3.set_xlabel("Temps (s)", fontweight="bold")
        ax3.grid(True, alpha=0.5)

        # ==========================================
        # FIGURE 2 : MOTORISATION & ÉNERGIE
        # ==========================================
        fig2 = plt.figure(figsize=(14, 8), constrained_layout=True)
        gs = fig2.add_gridspec(2, 2)  # Grille 2x2
        fig2.suptitle("Analyse Moteur & Consommation", fontsize=16, fontweight="bold")

        # A. Carte de fonctionnement Moteur (Scatter Plot)
        ax_map = fig2.add_subplot(gs[:, 0])  # Prend toute la colonne gauche

        # 1. Tracer la courbe max théorique (fond de carte)
        from utils.calculs import torque_map_rpm

        rpms_map = [p for p, t in torque_map_rpm]
        torques_map = [t for p, t in torque_map_rpm]
        # Interpolation lisse pour le tracé
        x_smooth = np.linspace(min(rpms_map), max(rpms_map), 200)
        y_smooth = [make_interpolated_function(r) for r in x_smooth]

        ax_map.plot(
            x_smooth, y_smooth, "k--", linewidth=2, label="Couple Max (Théorique)"
        )
        ax_map.fill_between(x_smooth, y_smooth, alpha=0.1, color="gray")

        # 2. Tracer les points de fonctionnement réels
        # On filtre les points où le moteur est allumé (RPM > 500 et Couple > 0)
        mask = (np.array(rpms) > 500) & (np.array(couples_utilises) > 0)
        sc = ax_map.scatter(
            np.array(rpms)[mask],
            np.array(couples_utilises)[mask],
            c=np.array(conso_l_100)[mask],  # Couleur selon la conso instantanée
            cmap="plasma",
            s=15,
            alpha=0.7,
            label="Points de fonctionnement",
        )
        plt.colorbar(sc, ax=ax_map, label="Conso Inst. (L/100km)")

        ax_map.set_xlabel("Régime Moteur (tr/min)", fontweight="bold")
        ax_map.set_ylabel("Couple Moteur (Nm)", fontweight="bold")
        ax_map.set_title("Points de Fonctionnement Moteur", fontweight="bold")
        ax_map.legend(loc="upper right")
        ax_map.grid(True)

        # B. Puissance Temporelle
        ax_power = fig2.add_subplot(gs[0, 1])
        ax_power.plot(temps, puissances_kw, color="tab:cyan", linewidth=1.5)
        ax_power.fill_between(temps, puissances_kw, alpha=0.2, color="tab:cyan")
        ax_power.set_ylabel("Puissance (kW)", fontweight="bold")
        ax_power.set_title("Puissance développée", fontweight="bold")
        ax_power.grid(True)

        # C. Consommation (Double Axe : L/100km et Litres Totaux)
        ax_conso = fig2.add_subplot(gs[1, 1], sharex=ax_power)

        # Axe Gauche : L/100km
        line1 = ax_conso.plot(
            temps,
            conso_l_100,
            color="tab:purple",
            linewidth=1,
            alpha=0.8,
            label="Inst. (L/100km)",
        )
        ax_conso.set_ylabel("L/100km", color="tab:purple", fontweight="bold")
        ax_conso.tick_params(axis="y", labelcolor="tab:purple")
        ax_conso.set_ylim(0, 50)  # Limite visuelle

        # Axe Droit : Litres cumulés
        ax_cumul = ax_conso.twinx()
        # Conversion kg -> Litres (densité ~0.75)
        litres_cumules = conso_cumul_kg / 0.75
        line2 = ax_cumul.plot(
            temps,
            litres_cumules,
            color="tab:brown",
            linewidth=2,
            linestyle="--",
            label="Cumul (L)",
        )
        ax_cumul.set_ylabel(
            "Consommation Totale (L)", color="tab:brown", fontweight="bold"
        )
        ax_cumul.tick_params(axis="y", labelcolor="tab:brown")

        ax_conso.set_xlabel("Temps (s)", fontweight="bold")
        ax_conso.set_title("Consommation de Carburant", fontweight="bold")
        ax_conso.grid(True)

        # Légende combinée
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax_conso.legend(lines, labels, loc="upper left")

        plt.show()
