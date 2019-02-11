"""

PowerIn - GPE_Power - KE_Power - Loss_Power = 0


Coordinates:

x - Spatial coordinate on horizontal plane
h(x) - Height of vehicle at location x
t - time

Derivatives:
x' - dx/dt
x'' - d^2/dt^2(x)

Definitions:

PowerIn - Power at wheels from propulsion = P
GPE_Power - Rate of change of GPE d/dt(mgh) = mg * dh/dt = mg * dh/dx * dx/dt = mg * x' * dh/dx
KE_Power - Rate of change of KE d/dt( 1/2 * m (x')^2) = m * x''


Initial Approximation:

P - mg * dh/dx * x' - m * x'' = 0

Let:

x_0 = x
x_1 = x'

x_0' = x' = x_1
x_1' = x'' = P / m - g * dh/dx * x_1

"""

class MS:
    def __init__(self):

        # State vector containing current state values: [x, x']
        self.state_history = [[1, 0]]
        self.t = [0]

    def Simulate(self):
        t_max = 10

        while self.t[-1] < t_max:
            self._step()

    def _step(self):
        """

        :return:
        """

        C = 0.5
        m = 1
        k = 10

        h = 0.01
        t_n = self.t[-1]
        x_n = self.state_history[-1]

        # Euler step
        x_np1_0 = x_n[0] + h * x_n[1]
        x_np1_1 = x_n[1] + h * (-1 * (C / m) * x_n[1] - (k / m) * x_n[0])
        t_np1 = t_n + h

        self.state_history.append([x_np1_0, x_np1_1])
        self.t.append(t_np1)


class QuarterModel:
    def __init__(self):

        # State vector containing current state values: [x_a, x_a', x_b, x_b']
        self.state_history = [[0, 0, -0.5, 0]]
        self.t = [0]

    def Simulate(self):
        t_max = 10

        while self.t[-1] < t_max:
            self._step()

    def _step(self):
        """

        :return:
        """

        C = 5
        m_1 = 500
        m_2 = 0.2
        k_1 = 10
        k_2 = 0.1

        h = 0.01
        t_n = self.t[-1]
        x_n = self.state_history[-1]

        # Euler step
        x_np1_0 = x_n[0] + h * x_n[1]
        x_np1_1 = x_n[1] + h * (-1 * (C / m_1) * (x_n[1] - x_n[3]) - (k_1 / m_1) * (x_n[0]-x_n[2]))
        x_np1_2 = x_n[2] + h * x_n[3]
        x_np1_3 = x_n[3] + h * ((C / m_2) * (x_n[1] - x_n[3]) + (k_1 / m_2) * (x_n[0] - x_n[2]) - (k_2 / m_2) * x_n[2])
        t_np1 = t_n + h

        self.state_history.append([x_np1_0, x_np1_1, x_np1_2, x_np1_3])
        self.t.append(t_np1)


if __name__ == '__main__':
    v = MS()
    v.Simulate()

    x_0 = [x[0] for x in v.state_history]
    x_1 = [x[1] for x in v.state_history]
    # x_2 = [x[2] for x in v.state_history]
    # x_3 = [x[3] for x in v.state_history]

    import matplotlib.pyplot as plt
    plt.plot(v.t, x_0)
    # plt.plot(v.t, x_2)
    plt.show()
    a = 0
