import numpy as np
"""
Problem:

y' = f(y, t)


k_1 = h * f(t_n, y_n)
k_2 = h * f(t_n + h/2, y_n + k_1/2)
k_3 = h * f(t_n + h/2, y_n + k_2/2)
k_4 = h * f(t_n + h, y_n + k_3)

y_n+1 = y_n + 1/6 * (k_1 + 2 * k_2 + 2 * k_3 + k_4)
t_n+1 = t + h


"""

class RK4:
    def solve(self, function, initial_conditions, time_step=0.01, time_range=[0, 1]):
        """

        :param functions: Reference to function that takes in state vector and returns state vector
        :param initial_conditions:
        :param time_step:
        :return:
        """

        # Initialise state space
        self.f = function
        self.y = [np.array(initial_conditions)]
        self.t = [time_range[0]]
        self.time_step = time_step

        while self.t[-1] <= time_range[1]:
            self._step()

        return self.t, self.y

    def _step(self):
        """

        :return:
        """

        h = self.time_step
        y_n = self.y[-1]
        t_n = self.t[-1]


        k_1 = np.multiply(h, self.f(t_n, y_n))
        k_2 = np.multiply(h, self.f(t_n + (h / 2.0), np.add(y_n, np.multiply((0.5), k_1))))
        k_3 = np.multiply(h, self.f(t_n + (h / 2.0), np.add(y_n, np.multiply((0.5), k_2))))
        k_4 = np.multiply(h, self.f(t_n + h, np.add(y_n, k_3)))

        # Apply weighting
        k_1_w = np.multiply(k_1, 1 / 6)
        k_2_w = np.multiply(k_2, 1 / 3)
        k_3_w = np.multiply(k_3, 1 / 3)
        k_4_w = np.multiply(k_4, 1 / 6)

        y_np1 = np.add(y_n, np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, k_4_w))))
        # y_np1 = np.add(y_n, k_1)
        t_np1 = t_n + h

        self.y.append(y_np1)
        self.t.append(t_np1)

def MassSpringDamper(t, y):

    C = 0.4
    m = 1
    k = 1

    f = [
        y[1],
        # (-1) * (k / m) * y[0]
        -1 * (C / m) * y[1] - (k / m) * y[0]
    ]

    return f

def Pendulum(t, y):

    l = 1
    m = 1
    g = 9.81
    b = 0.1

    f = [
        y[1],
        # (-1) * (g / l) * np.sin(y[0])
        -1 * (b / m) * y[1] - (g / l) * np.sin(y[0])
    ]

    return f

if __name__ == '__main__':




    solver = RK4()
    result = solver.solve(Pendulum, [1, 0], time_range=[0, 10], time_step=0.05)

    t = result[0]
    x = [x[0] for x in result[1]]

    import matplotlib.pyplot as plt

    plt.plot(t, x)
    plt.show()

    print(x)
    print(t)

    # from scipy.integrate import odeint
    #
    # t = np.linspace(0, 10, 101)
    #
    # sol = odeint(Pendulum, [1, 0], t, tfirst=True)
    #
    # x = [x[0] for x in sol]
    # print(sol)
    #
    # plt.plot(t, x)
    # plt.show()
    #
    # print(4)