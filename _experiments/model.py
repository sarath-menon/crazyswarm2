import numpy as np
from numpy.polynomial import Polynomial
import matplotlib.pyplot as plt


positions = np.linspace(0, 10, 20)
velocities = np.sin(positions)
d = 9

p = Polynomial.fit(positions, velocities, d)
print(p)

v = p.deriv()
print(v)

a = v.deriv()
print(a)

plt.plot(positions, velocities, label="velocities")
plt.plot(positions, p(positions), label="polynomial")
plt.plot(positions, v(positions), label="velocity")
plt.plot(positions, a(positions), label="acceleration")
plt.legend()
plt.show()