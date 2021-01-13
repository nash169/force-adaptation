#! /usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
import sys

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from utils import get_data

data = get_data(sys.argv[1], "plane", "surface", "circle", "force")


fig1 = plt.figure()
ax = fig1.add_subplot(111, projection="3d")

# Define colors based on function value
surface_force = data["force"][:, -1].reshape(100, 100)
norm = matplotlib.colors.SymLogNorm(
    1, vmin=surface_force.min(), vmax=surface_force.max())
colors = cm.jet(norm(surface_force))

# Surface
surf = ax.plot_surface(
    data["plane"][:, 0].reshape(100, 100),
    data["plane"][:, 1].reshape(100, 100),
    data["plane"][:, 2].reshape(100, 100), facecolors=colors, linewidth=0, alpha=0.5)
ax.set_xlim3d(-10, 10)
ax.set_ylim3d(-10, 10)
ax.set_zlim3d(-10, 10)

# Colorbar
sm = plt.cm.ScalarMappable(cmap=cm.jet, norm=norm)
sm.set_array(surface_force)
fig1.colorbar(sm, shrink=0.5, aspect=5)

# Circle
ax.plot(data["circle"][:, 0], data["circle"]
        [:, 1], data["circle"][:, 2], color='b', linewidth=3)

ax.plot_surface(data["surface"][:, 0].reshape(100, 100), data["surface"][:, 1].reshape(
    100, 100), data["surface"][:, 2].reshape(100, 100), cmap=cm.jet)

# Motion
# ax.plot(data["dynamics"][:, 0], data["dynamics"][:, 1], data["dynamics"][:, 2])

# fig2 = plt.figure()
# ax = fig2.add_subplot(111, projection="3d")
# ax.plot_surface(data["surface"][:, 0].reshape(100, 100), data["surface"][:, 1].reshape(
#     100, 100), data["surface"][:, 2].reshape(100, 100), cmap=cm.jet)

# ax.set_xlim3d(-10, 10)
# ax.set_ylim3d(-10, 10)
# ax.set_zlim3d(-10, 10)

# # 3D load distribution
# fig2 = plt.figure()
# ax = fig2.add_subplot(111, projection="3d")
# ax.plot_surface(data["force"][:, 0].reshape(100, 100), data["force"][:, 1].reshape(
#     100, 100), data["force"][:, 2].reshape(100, 100), cmap=cm.jet)

# # Vector Field
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.quiver(data["dynamics"][:, 0].reshape(20, 20, 20), data["dynamics"][:, 1].reshape(20, 20, 20), data["dynamics"][:, 2].reshape(20, 20, 20),
#           data["dynamics"][:, 0].reshape(20, 20, 20), data["dynamics"][:, 1].reshape(20, 20, 20), data["dynamics"][:, 2].reshape(20, 20, 20), length=0.1)

plt.show()
