from src.Optimisation.Tests.Test_Optimiser import Polynomial


import numpy as np


print(np.random.rand(1)[0])


q = Polynomial([1, 0, 0, 0, 1])

kjh = [-3, -2, -1, 0,  1, 2, 3]
for i in kjh:
    q.x = [i]
    print(q.cost())

a = 4



