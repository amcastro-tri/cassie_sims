#!/usr/bin/env python3

from pydrake.all import *
import matplotlib.pyplot as plt
import numpy as np
from math import *

plant = MultibodyPlant(0.001)

print(f'bodies = {plant.num_bodies()}')

nb = plant.num_bodies()

print('bodies = {}'.format(nb))

x = np.linspace(0, pi, 50)
y = np.sin(x)

# plot
fig, ax = plt.subplots()

ax.plot(x, y, linewidth=2.0)

ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
       ylim=(0, 8), yticks=np.arange(1, 8))

plt.show()

