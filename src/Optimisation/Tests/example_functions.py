import numpy as np

class Polynomial:
    def __init__(self, coefficients=[0, 0, 1]):
        self.coefficients = coefficients

        self.x = [0]

    def cost(self):
        x = self.x[0]
        y = 0
        for index, value in enumerate(self.coefficients):

            y += np.power(x, index) * value

        return y

class XY:
    def __init__(self, coefficients=[0, 0, 1]):
        self.coefficients = coefficients

        self.x = [0]
        self.y = [0]

    def cost(self):

        return self.x[0] * self.x[0] + self.y[0] * self.y[0]


