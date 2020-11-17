#! /usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import sys

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from utils import get_data

data = get_data(sys.argv[1], "X", "Y", "F")

fig = plt.figure()
ax = fig.add_subplot(121, projection="3d")
surf = ax.plot_surface(
    data["X"], data["Y"], data["F"], cmap=cm.Spectral, linewidth=0, antialiased=False
)
fig.colorbar(surf, ax=ax)
ax = fig.add_subplot(122)
contour = ax.contourf(data["X"], data["Y"], data["F"], cmap=cm.Spectral)
ax.set_aspect("equal", "box")
fig.colorbar(contour, ax=ax)

plt.show()
