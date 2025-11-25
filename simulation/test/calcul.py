import unittest

from utils.calculs import calcul_aero_drag


class TestCalcul(unittest.TestCase):
    def test_calcul_aero_drag(self):
        rho = 1.225  # densité de l’air kg/m³
        S = 2.0  # surface frontale m²
        C_d = 0.3  # coefficient de traînée réaliste

        # Test 1 : vitesse 30 m/s
        v = 30
        expected = 0.5 * rho * S * C_d * v**2  # 0.5 * 1.225 * 2.0 * 0.3 * 900
        self.assertAlmostEqual(calcul_aero_drag(rho, C_d, S, v), expected, places=6)

        # Test 2 : vitesse 0 m/s
        v = 0
        expected = 0.0
        self.assertAlmostEqual(calcul_aero_drag(rho, C_d, S, v), expected, places=6)

        # Test 3 : vitesse 15 m/s
        v = 15
        expected = 0.5 * rho * S * C_d * v**2
        self.assertAlmostEqual(calcul_aero_drag(rho, C_d, S, v), expected, places=6)

        # Test 4 : vitesse négative
        v = -20
        expected = 0.5 * rho * S * C_d * v**2
        self.assertAlmostEqual(calcul_aero_drag(rho, C_d, S, v), expected, places=6)


if __name__ == "__main__":
    unittest.main()
