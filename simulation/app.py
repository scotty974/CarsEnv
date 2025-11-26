import random

from environment.env import Environment

env = Environment(0.38, 1.225, 2.29, 0.012, 1843, 0.0, 3.5, 0.345)
print("\nParamètres du véhicule:")
print(f"  - Masse: {env.mass} kg")
print(f"  - Puissance max: {env.P_max / 1000} kW ({env.P_max / 1000 * 1.36:.0f} ch)")
print(f"  - Vitesse max: {env.v_max:.1f} m/s ({env.v_max * 3.6:.0f} km/h)")
print(f"  - Cx: {env.drag_coefficient}")
print(f"  - Surface frontale: {env.surface_frontal_area} m²")
print("\n" + "=" * 70)

actions_urbain = (
    [1] * 15  # Accélération départ
    + [1] * 10  # Maintien 30 km/h
    + [0] * 15  # Freinage feu rouge
    + [1] * 5  # Redémarrage
    + [1] * 12  # Maintien
    + [0] * 15  # Freinage
    + [1] * 10  # Accélération
    + [1] * 5  # Maintien
    + [0] * 25  # Freinage final
)

env.reset_state()
for i in actions_urbain:
    env.step(i)

env.plot_data()
v_final, distance, _, _, conso = env.state_min

print(f"\nRésultats après {len(actions_urbain)} secondes:")
print(f"  - Distance parcourue: {distance:.0f} m ({distance / 1000:.2f} km)")
print(f"  - Vitesse finale: {v_final * 3.6:.1f} km/h")
print(f"  - Consommation: {conso * 1000:.2f} grammes")
print(f"  - Consommation au 100 km: {(conso / (distance / 100000)):.2f} L/100km")
print(f"    (équivalent essence, densité ~0.75 kg/L)")
