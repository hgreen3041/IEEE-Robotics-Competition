import matplotlib.pyplot as plt
import numpy as np

# Sample data
x = np.linspace(0, 99, 31)
y = [.01,0.4,.73,1.12,1.4,1.7,1.94,2.2,2.45,2.64,2.87,3.03, 3.18,3.36,3.5,3.65,3.77,3.9,4,4.1,4.2,4.3,4.45,4.57,4.67,4.76,4.86,4.95,5.06,5.13,5.22, ]

# Scatter plotting the data points
plt.scatter(x, y, color='red', marker='o')  # 'o' specifies circular markers
plt.xlabel('Time(s)')
plt.ylabel('Capacitor Voltage(V)')
plt.title('Capacitor Charging')
plt.grid(True)
plt.show()
