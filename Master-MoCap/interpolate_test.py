from scipy import interpolate
import numpy as np


x = np.linspace(-7, 7, 15)

y = np.linspace(5, -5, 11)

xv, yv = np.meshgrid(x, y)

print(xv)
print(yv)
