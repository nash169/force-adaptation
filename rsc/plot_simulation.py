#! /usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
import sys

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from utils import get_data

data = get_data(sys.argv[1], "time", "state", "action", "plane", "circle")

fig = plt.figure()
ax = fig.gca(projection="3d")
ax.plot(data["state"][:, 0], data["state"]
        [:, 1], data["state"][:, 2], color='r', linewidth=1)

surf = ax.plot_surface(
    data["plane"][:, 0].reshape(100, 100),
    data["plane"][:, 1].reshape(100, 100),
    data["plane"][:, 2].reshape(100, 100), linewidth=0, alpha=0.5)

ax.plot(data["circle"][:, 0], data["circle"]
        [:, 1], data["circle"][:, 2], color='b', linewidth=3)

ax.set_xlim3d(-1, 1)
ax.set_ylim3d(-1, 1)
ax.set_zlim3d(-1, 1)


fig = plt.figure()
ax = fig.gca()
ax.plot(data["time"], data["action"][:, 0], color='r', linewidth=3)
ax.plot(data["time"], data["action"][:, 1], color='g', linewidth=3)
ax.plot(data["time"], data["action"][:, 2], color='b', linewidth=3)

plt.show()
