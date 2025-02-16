import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('/Users/aryalabroo/Documents/Engineering/lander1/info.txt')
plt.figure(1)
plt.clf()
plt.xlabel('altitude (m)')
plt.ylabel('velocity (m/s)')
plt.grid()
plt.plot(results[:, 0], -results[:, 1], label='actual')
plt.plot(results[:, 0], -results[:, 2], label='target')
plt.title('kh = 0.05, kp = 1, delta = 0.5, 200km descent')
plt.legend()
plt.show()