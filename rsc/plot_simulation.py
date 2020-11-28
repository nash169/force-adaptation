#! /usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
import sys

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from utils import get_data

data = get_data(sys.argv[1], "time", "state", "action")

fig = plt.figure()
ax = fig.gca(projection="3d")
ax.plot(data["state"][:, 0], data["state"]
        [:, 1], data["state"][:, 2], color='b', linewidth=3)

fig = plt.figure()
ax = fig.gca()
ax.plot(data["time"], data["action"][:, 0], color='r', linewidth=3)
ax.plot(data["time"], data["action"][:, 1], color='g', linewidth=3)
ax.plot(data["time"], data["action"][:, 2], color='b', linewidth=3)

plt.show()
