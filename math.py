from matplotlib import pyplot, rcParams
from math import sin, cos, pi

rcParams['figure.figsize']=(12,6)

x = [0.01*i for i in range(-200,201)]

y_sin = [sin(j*pi)for j in x]

pyplot.plot(x,y_sin)
pyplot.show()
