import csv
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import minimize
from scipy.optimize import approx_fprime

cycle_file = "/Users/tom/Documents/University/Y3_S2/SG3/WLTP_Class1.csv"

t = []
v = []

with open(cycle_file, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')

    for row_index, row in enumerate(reader):
        if row_index > 0:
            t.append(float(row[0]))
            v.append(float(row[1]) * (1/3.6))


def WLTC(t, v, m, Cd, A, Crr):
    """

    :param t:
    :param v:
    :param vehicle_parameters:
    :return:
    """


    energy = [None] * (len(t) - 1)
    s = [None] * (len(t) - 1)

    # Calculate Forces
    for i in range(len(t) - 1):

        dt = t[i+1] - t[i]
        v_avg = (v[i+1] + v[i]) / 2

        # Inertial
        dp_dt = ((v[i+1] - v[i]) / dt) * m
        if dp_dt < 0:
            dp_dt = 0

        # Aerodynamic
        F_a = 0.5 * 1.225 * A * Cd * v_avg * v_avg

        # Rolling resistance
        F_rr = m * 9.81 * Crr

        # Distance travelled
        s[i] = v_avg * dt

        # Energy
        energy[i] = s[i] * (dp_dt + F_a + F_rr)

    s_total = sum(s)
    energy_total = sum(energy)

    km_kwh = s_total / (energy_total / 3600)

    return km_kwh



def WLTCSensitivity(input_vec, *args):

    m = input_vec[0]
    Cd = input_vec[1]
    A = input_vec[2]
    Crr = input_vec[3]

    t = args[0]
    v = args[1]

    return WLTC(t, v, m, Cd, A, Crr)


def Spacing(range, n):
    return (range[1] - range[0]) / n


def Cd_m_data(m_range, Cd_range, A, Crr, t, v, desired_range=300, battery_density=5):
    """

    :param m_range:
    :param Cd_range:
    :param A:
    :param Crr:
    :param t:
    :param v:
    :return:
    """

    n = 10
    dm = Spacing(m_range, n)
    dCd = Spacing(Cd_range, n)

    data = [None] * (n + 1)
    for row in range(len(data)):
        data[row] = [None] * (n + 1)

    m_list = [m_range[0] + i * dm for i in range(n + 1)]
    Cd_list = [Cd_range[0] + j * dCd for j in range(n + 1)]

    for i in range(n + 1):
        m = m_list[i]
        print(m)

        for j in range(n + 1):
            Cd = Cd_list[j]
            print(Cd)

            x0 = [1]
            res = minimize(BattSize, x0, args=(t, v, desired_range, m, Cd, A, Crr, battery_density))
            batt_energy = res['x'][0]
            batt_mass = battery_density * batt_energy

            data[i][j] = WLTC(t, v, m + batt_mass, Cd,  A, Crr)

    return m_list, Cd_list, data

def Cd_A_data(Cd_range, A_range, m, Crr, t, v):

    n = 25
    dA = Spacing(A_range, n)
    dCd = Spacing(Cd_range, n)

    data = [None] * (n + 1)
    for row in range(len(data)):
        data[row] = [None] * (n + 1)

    A_list = [A_range[0] + i * dA for i in range(n + 1)]
    Cd_list = [Cd_range[0] + j * dCd for j in range(n + 1)]

    for i in range(n + 1):
        A = A_list[i]
        print(A)

        for j in range(n + 1):
            Cd = Cd_list[j]
            print(Cd)

            data[i][j] = WLTC(t, v, m, Cd, A, Crr)

    return A_list, Cd_list, data


def FourAxPlot():
    fig2D = plt.figure()
    ax = [
        fig2D.add_subplot(221),
        fig2D.add_subplot(222),
        fig2D.add_subplot(223),
        fig2D.add_subplot(224)]

    areas = [0.75, 1, 1.25, 1.5]

    m_range = [80, 200]
    Cd_range = [0.05, 0.5]

    for i in range(4):
        m, Cd, data = Cd_m_data(m_range, Cd_range, areas[i], 0.005, t, v)
        cont = ax[i].contour(m,  Cd, data, 15)
        ax[i].clabel(cont, inline=1, fontsize=10)
        ax[i].set_title('A = ' + str(areas[i]))

    plt.show()

def BattSize(battery_energy, *args):

    t = args[0]
    v = args[1]
    desired_range = args[2]

    base_mass_inc_person = args[3]
    Cd = args[4]
    A = args[5]
    Crr = args[6]
    battery_density = args[7]

    battery_mass = battery_density * battery_energy[0]

    m = battery_mass + base_mass_inc_person

    efficiency = WLTC(t, v, m, Cd, A, Crr)
    range_result = efficiency * battery_energy[0] *  0.7

    print('Energy: ' + str(battery_energy[0]) + ', Mass: ' + str(battery_mass) + ', Range: ' + str(range_result))

    return np.abs(range_result - desired_range)


def CarSize(t, v, desired_range, base_mass=90, person_mass=75, battery_density=5, Cd=0.2, A=1.26, Crr=0.02):
    """

    :param t:
    :param v:
    :return:
    """

    x0 = [1]

    res = minimize(BattSize, x0, args=(t, v, desired_range, base_mass + person_mass, Cd, A, Crr, battery_density))

    batt_energy = res['x'][0]
    batt_mass = battery_density * batt_energy

    return batt_energy, batt_mass


base_mass = 230
person_mass = 70

batt_energy, batt_mass = CarSize(t, v, 112, base_mass=base_mass, person_mass=person_mass, battery_density=5, Cd=0.2, A=1.26, Crr=0.0015)

total_mass = base_mass + person_mass + batt_mass

res = WLTC(t, v, total_mass, 0.2, 1.26, 0.0015)

print()

range_list = np.linspace(10, 1000, 10)
batt_mass_list = [CarSize(t, v, r)[1] for r in range_list]

fig, ax_1 = plt.subplots()
ax_1.plot(range_list,batt_mass_list)
ax_1.scatter(300, 33.159, color=[1,0,0])
ax_1.set_title('Mass Of Battery For Given Range')
ax_1.set_xlabel('Range (km)')
ax_1.set_ylabel('Mass of Battery (kg)')
ax_1.grid()


fig, ax = plt.subplots()
m_range = [80, 200]
Cd_range = [0.05, 0.5]

m, Cd, data = Cd_m_data(m_range, Cd_range, 1.26, 0.02, t, v)
cont = ax.contour(m,  Cd, data, 20)
ax.clabel(cont, inline=1, fontsize=8)
ax.set_title('Estimated Efficiency (km / kWh)')

ax.set_xlabel('Mass Including Driver (kg)')
ax.set_ylabel('Coefficient Of Drag')


ax.scatter(150, 0.2, color=[1,0,0])
# ax.scatter(175, 0.2, color=[0,0,1])
ax.plot([80, 150], [0.2, 0.2], color=[0.1, 0.1, 0.1], linestyle='--', alpha=0.25)
ax.plot([150, 150], [0.05, 0.2], color=[0.1, 0.1, 0.1], linestyle='--', alpha=0.25)


# Sensitivity
m = 150
Cd = 0.2
A = 1.26
Crr = 0.02

gradient = approx_fprime([m, Cd, A, Crr], WLTCSensitivity, 0.001, t, v)

dn_dm = gradient[0]
dn_dCd = gradient[1]
dn_dA = gradient[2]
dn_dCrr = gradient[3]

delta_1p_m = (m * 0.01) * dn_dm
delta_1p_Cd = (Cd * 0.01) * dn_dCd
delta_1p_A = (A * 0.01) * dn_dA
delta_1p_Crr = (Crr * 0.01) * dn_dCrr

print(delta_1p_m, delta_1p_Cd, delta_1p_A, delta_1p_Crr)


plt.show()


