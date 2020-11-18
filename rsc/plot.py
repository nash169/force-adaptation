#! /usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import sys

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from utils import get_data

data = get_data(sys.argv[1], "plane", "circle")

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(data["circle"][:, 3], data["circle"]
        [:, 4], data["circle"][:, 5], color='b', linewidth=3)

surf = ax.plot_surface(
    data["plane"][:, 2].reshape(100, 100),
    data["plane"][:, 3].reshape(100, 100),
    data["plane"][:, 4].reshape(100, 100), cmap=cm.Spectral, alpha=0.5, antialiased=False
)

ax.set_xlim3d(-10, 10)
ax.set_ylim3d(-10, 10)

plt.show()
