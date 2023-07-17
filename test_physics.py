"""

created by travis tran on July 13, 2023

made to test physics.py

"""

import unittest
import physics
import numpy as np


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
        self.assertNotEqual(physics.calculate_acceleration(10, 10), 321)
        self.assertRaises(ValueError, physics.calculate_acceleration, 10, -1000)

    def test_calculate_angular_acceleration(self):
        self.assertEqual(physics.calculate_angular_acceleration(10, 10), 1)
        self.assertNotEqual(physics.calculate_angular_acceleration(10, 10), 4905)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, 10, -1000)

    def test_calculate_torque(self):
        self.assertEqual(physics.calculate_torque(10, 0, 10), 100)
        self.assertNotEqual(physics.calculate_torque(10, 0, 10), 49)
        self.assertRaises(ValueError, physics.calculate_torque, 10, 10, -1000)

    def test_calculate_moment_of_inertia(self):
        self.assertEqual(physics.calculate_moment_of_inertia(10, 10), 1000)
        self.assertNotEqual(physics.calculate_moment_of_inertia(10, 10), 1001)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, -10, -1000)

    def test_calculate_auv_acceleration_testing(self):
        self.assertEqual(
            physics.calculate_auv_acceleration(100, np.pi / 6, 100, 0.1, 0.5),
            1.1547005383792515
        )
        self.assertNotEqual(
            physics.calculate_auv_acceleration(100, np.pi / 6, 100, 0.1, 0.5), 19
        )
        self.assertRaises(
            ValueError,
            physics.calculate_auv_acceleration,
            101,
            np.pi,
            100,
            0.1,
            0.5
        )


    def test_calculate_auv_angular_acceleration(self):
        self.assertEqual(
            physics.calculate_auv_angluar_acceleration(100, np.pi / 6, 1, 0.5),
            43.30127018922194
        )
        self.assertNotEqual(
            physics.calculate_auv_angluar_acceleration(100, np.pi / 6, 1, 0.5), 19
        )
        self.assertRaises(
            ValueError,
            physics.calculate_auv_angluar_acceleration,
            101,
            np.pi,
            1,
            0.5
        )


    def test_calculate_auv2_acceleration(self):
        test = np.array([1,3,1,2])
        wrongTest = np.array([-10,1,1,1])
        ans = np.array([0.008253356149096785, -0.0056464247339503525])
        wrongAns = np.array([1,1])
        self.assertEqual(
            physics.calculate_auv2_acceleration(test, 0.5, 0.3, 100)[0], ans[0]
        )
        self.assertEqual(
            physics.calculate_auv2_acceleration(test, 0.5, 0.3, 100)[1], ans[1]
        )
        self.assertNotEqual(
            physics.calculate_auv2_acceleration(test, 0.5, 0.3, 100)[0], wrongAns[0]
        )
        self.assertNotEqual(
            physics.calculate_auv2_acceleration(test, 0.5, 0.3, 100)[1], wrongAns[1]
        )
        self.assertRaises(
            ValueError,
            physics.calculate_auv2_acceleration,
            wrongTest,
            np.pi,
            1,
            0.5
        )

    def test_calculate_auv2_angular_acceleration(self):
        test = np.array([1,3,1,2])
        wrongTest = np.array([-10,1,1,1])
        self.assertEqual(
            physics.calculate_auv2_angular_acceleration(test, 0.5, 1.5, 1.8),
            6.538019436969373
        )
        self.assertNotEqual(
            physics.calculate_auv2_angular_acceleration(test, 0.5, 1.5, 1.8), 19
        )
        self.assertRaises(
            ValueError,
            physics.calculate_auv2_angular_acceleration,
            wrongTest,
            np.pi,
            1,
            0.5
        )



if __name__ == "__main__":
    unittest.main()
