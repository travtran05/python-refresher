import unittest
import BankAcc


class TestAcc(unittest.TestCase):
    def test_acc(self):
        travis = BankAcc.BankAcc("Travis", 100, 0)
        self.assertEqual(travis.name, "Travis")


if __name__ == "__main__":
    unittest.main()
