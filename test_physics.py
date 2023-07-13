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
        self.assertEqual(physics.calculate_pressure(1), 9810)
        self.assertEqual(physics.calculate_pressure(0.5), 4905)
        self.assertRaises(ValueError, physics.calculate_pressure, -1000)


if __name__ == "__main__":
    unittest.main()
