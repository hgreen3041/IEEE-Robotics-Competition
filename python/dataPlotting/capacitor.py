import matplotlib.pyplot as plt
import numpy as np

# Sample data
t = [0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60]
v = [.011,0.59,1.15,1.7,2.15,2.65,3.06,3.38,3.71,4.01,4.21,4.48,4.68,4.92,5.14,5.31,5.51,5.7,5.86,6.04,6.21,6.35,6.51,6.65,6.78,6.93,7.07,7.19,7.32,7.45,7.58 ]

# Scatter plotting the data points
plt.figure(1)
plt.scatter(t, v, color='red', marker='o')  # 'o' specifies circular markers
plt.plot(t, v, linestyle='-', color='r')
plt.axhline(y=2.7, label="System Start-Up Voltage",linestyle='--', linewidth=2)
plt.axhline(y=1.4, label="System Shut-Down Voltage", color='y',linestyle='--', linewidth=2)
plt.axhline(y=4.58,label="Goal(~5s of Drive Time)", color='g',linestyle='--', linewidth=2)
plt.xlabel('Time(s)', fontsize=16)
plt.ylabel('Voltage(V)', fontsize=16)
plt.title('Capacitor Voltage', fontsize=20)
plt.grid(True)
plt.legend(fontsize="large")

C = 2.5
shutDownEnergy = .5*(C)*1.4**2

E = .5*C*np.power(v,2)

plt.figure(2)
plt.scatter(t, E, color='red', marker='o')
plt.plot(t, E, linestyle='-', color='r')
plt.axhline(y=22.2,label="Goal(~5s of Drive Time)", color='g',linestyle='--', linewidth=2)
plt.xlabel('Time(s)', fontsize=16)
plt.ylabel('Energy(J)', fontsize=16)
plt.title('Capacitor Energy', fontsize=20)
plt.grid(True)
plt.legend(fontsize="large")

plt.show()
