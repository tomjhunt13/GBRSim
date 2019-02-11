import math

class Vehicle:
    def __init__(self):

        # Vehicle attributes
        self.m = 80         # Total vehicle mass (kg)
        self.Crr = 0.005    # Coefficient of rolling resistance
        self.Cd = 0.3       # Coefficient of drag
        self.A = 1          # Frontal area (m^2)



        # State vector containing current state values: [x, x']
        self.state_history = [[5, 2]]
        self.t = [0]

    def simulate(self, terrain):

        self.terrain = terrain
        self.terrain_gradient = [None] * (len(self.terrain[0]) - 1)

        for index in range(len(self.terrain_gradient)):
            delta_y = self.terrain[1][index + 1] - self.terrain[1][index]
            delta_x = self.terrain[0][index + 1] - self.terrain[0][index]

            self.terrain_gradient[index] = math.tan(delta_y / delta_x)

        while self.state_history[-1][0] > self.terrain[0][0] and self.state_history[-1][0] < self.terrain[0][-1]:
            self._step()

    def gradient(self, x):
        return getElementInSortedList(x, self.terrain[0][:-1], self.terrain_gradient)

    def _step(self):
        """

        :return:
        """

        """
        P - force exerted on vehicle by tyres
        theta - current gradient
        """
        g = 9.81
        h = 0.01

        t_n = self.t[-1]
        x_n = self.state_history[-1]

        grad = self.gradient(x_n[0])
        P = 1 * math.sin(t_n * math.pi / 8)
        # P = 0

        # Euler step
        x_np1_0 = x_n[0] + h * x_n[1]
        x_np1_1 = x_n[1] + h * ((P / self.mass) * math.cos(grad) - g * math.sin(grad))
        t_np1 = t_n + h

        self.state_history.append([x_np1_0, x_np1_1])
        self.t.append(t_np1)



def getElementInSortedList(x, list_x, list_y):

    if x < list_x[0]:
        return list_y[0]

    index = 0
    while list_x[index] < x:
        index += 1

        if index > len(list_x) - 1:
            return list_y[-1]

    return list_y[index]


if __name__ == '__main__':
    # terrain = [[0, 0], [5, 1.25], [10, 2], [15, 1], [20, 0]]
    terrain_x = [0, 5, 10, 15, 20]
    terrain_y = [0, 1.25, 2, 1, 0]
    terrain = [terrain_x, terrain_y]

    v = Vehicle()
    v.simulate(terrain)

    x_0 = [x[0] for x in v.state_history]
    x_1 = [x[1] for x in v.state_history]

    import matplotlib.pyplot as plt
    plt.plot(terrain_x, terrain_y)

    plt.plot(v.t, x_0)
    plt.plot(v.t, x_1)

    plt.show()
    a = 0
