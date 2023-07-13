import unittest
import BankAcc


class TestAcc(unittest.TestCase):
    def test_acc(self):
        travis = BankAcc.BankAcc("Travis", 100, 0)
        self.assertEqual(travis.name, "Travis")
        self.assertEqual(travis.accNumber, 100)
        self.assertEqual(travis.balance, 0)


if __name__ == "__main__":
    unittest.main()
