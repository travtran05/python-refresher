"""

created by travis tran on July 13, 2023

made to test physics.py

"""

import unittest
import physics


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(1, 1000), 9810)
        self.assertRaises(ValueError, physics.calculate_buoyancy, 1, -1000)
        self.assertEqual(physics.calculate_buoyancy(1000, 10000000), 98100000000)

    def test_will_it_float(self):
        self.assertEqual(physics.will_it_float(10, 10), True)
        self.assertEqual(physics.will_it_float(1, 9000), False)
        self.assertRaises(ValueError, physics.will_it_float, 1, -1000)

    def test_calculate_pressure(self):
        self.assertEqual(physics.calculate_pressure(1), 9810 + 101325)
        self.assertEqual(physics.calculate_pressure(0.5), 4905 + 101325)
        self.assertRaises(ValueError, physics.calculate_pressure, -1000)

    # 7/14

    def test_calculate_acceleration(self):
        self.assertEqual(physics.calculate_acceleration(10, 10), 1)
        self.assertFalse(physics.calculate_acceleration(10, 10), 321)
        self.assertRaises(ValueError, physics.calculate_acceleration, 10, -1000)

    def test_calculate_angular_acceleration(self):
        self.assertEqual(physics.calculate_angular_acceleration(10, 10), 1)
        self.assertFalse(physics.calculate_angular_acceleration(10, 10), 4905)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, 10, -1000)

    def test_calculate_torque(self):
        self.assertEqual(physics.calculate_torque(10, 0, 10), 100)
        self.assertFalse(physics.calculate_torque(10, 0, 10), 49)
        self.assertRaises(ValueError, physics.calculate_torque, 10, 10 - 1000)

    def test_calculate_moment_of_inertia(self):
        self.assertEqual(physics.calculate_moment_of_inertia(10, 10), 1000)
        self.assertFalse(physics.calculate_moment_of_inertia(10, 10), 1001)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, -10, -1000)


if __name__ == "__main__":
    unittest.main()
