class BankAcc:
    def __init__(self, balance, name, accNumber):
        self.balance = balance
        self.name = name
        self.accNumber = accNumber

    def deposit(self, number):
        self.balance = self.balance+number

    def withdraw(self, number):
        if number>self.balance:
            print('You do not have enough balance.')
        else:
            self.balance = self.balance-number
    
    def giveBalance(self):
        print(str(self.balance))