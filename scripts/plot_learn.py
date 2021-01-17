#! /usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
import sys

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from utils import get_data

data = get_data(sys.argv[1], "time", "state", "action",
                "force_measured", "force_adaptation", "plane", "circle", "scatter")

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

ax.scatter(data["scatter"][:, 0], data["scatter"][:, 1],
           data["scatter"][:, 2], color='r')


fig = plt.figure()
ax = fig.gca()
ax.plot(data["time"], data["force_measured"][:, 2], color='r', linewidth=1)
ax.plot(data["time"], data["force_adaptation"], color='g', linewidth=1)
# ax.plot(data["time"], 10, color='b', linewidth=3)
# ax.set_ylim(0, 15)

plt.show()
