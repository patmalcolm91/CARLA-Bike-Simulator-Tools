"""
Draw plots visualizing the display setup.
"""

import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import math
import yaml
import argparse

COLORS = ["blue", "orange", "green", "red", "purple", "yellow", "gray"]


def plot_displays(configs, ax=None):
    """
    Plot the arrangement of monitors/displays.

    :param configs: dict of display config dicts
    :type configs: dict
    :param ax: matplotlib Axes object on which to plot
    :type ax: plt.Axes
    """
    if ax is None:
        ax = plt.gca()
    ax.invert_yaxis()
    x_landmarks = []
    y_landmarks = []
    names = sorted(list(configs.keys()))
    for name, cfg in configs.items():
        i = names.index(name)
        x, y = cfg["window_pos"]
        width, height = cfg["display_size"]
        x_landmarks += [x, x+width]
        y_landmarks += [y, y+height]
        rect = Rectangle((x, y), width, height, fill=True, edgecolor="black", facecolor=(*to_rgb(COLORS[i]), 0.3), label=name)
        ax.add_patch(rect)
        ds_label = "%dx%d" % cfg["display_size"]
        res_label = "(%dx%d)" % cfg["resolution"]
        label = "\n".join([name, ds_label, res_label])
        plt.annotate(label, (0.5, 0.5), xycoords=rect, ha="center", va="center")
    x_landmarks = list(set(x_landmarks))
    y_landmarks = list(set(y_landmarks))
    ax.set_xticks(x_landmarks)
    ax.set_yticks(y_landmarks)
    ax.set_aspect("equal")
    # ax.legend()
    ax.autoscale()


def plot_fov(configs, ax=None, show_position=False):
    """
    Plot the fields of view (FOVs) of the displays.

    :param configs: dict of display config dicts
    :type configs: dict
    :param ax: matplotlib Axes object on which to plot
    :type ax: plt.Axes
    """
    if ax is None:
        ax = plt.subplot(projection="polar")
    names = sorted(list(configs.keys()))
    landmarks = []
    for name, cfg in configs.items():
        i = names.index(name)
        x, y, z = cfg["camera_x"], cfg["camera_y"], cfg["camera_z"]
        r0, theta0 = 0, 0
        if show_position:
            r0 = math.sqrt(x**2 + y**2)
            theta0 = math.atan2(y, x)
        c = cfg["camera_yaw"]
        fov = cfg["fov"]
        a1, a2 = c-fov/2, c+fov/2
        landmarks += [a1, c, a2]
        c *= math.pi / 180
        a1 *= math.pi / 180
        a2 *= math.pi / 180
        r = 1
        l0 = Line2D([theta0, theta0+c], [r0, r0+r*math.cos(fov/2*math.pi/180)], color=COLORS[i], dashes=(5, 2, 2, 2))
        l1 = Line2D([theta0, theta0+a1], [r0, r0+r], color="black")
        l2 = Line2D([theta0, theta0+a2], [r0, r0+r], color="black")
        l3 = Line2D([theta0+a1, theta0+a2], [r0+r, r0+r], color=COLORS[i])
        ax.add_artist(l1)
        ax.add_artist(l2)
        ax.add_artist(l0)
        ax.add_artist(l3)
        label_angle = (c*180/math.pi+270)%180-90
        label = name + "\n(" + str(fov) + "°)"
        plt.annotate(label, (0.5, 0.5), xycoords=l0, ha="center", va="center", rotation=label_angle,
                     backgroundcolor="white", color=COLORS[i])
    landmarks = list(set(landmarks))
    for theta in landmarks:
        label_angle = (theta + 270) % 180 - 90
        plt.annotate(str(theta)+"°", (theta*math.pi/180, r*0.8), ha="center", va="center", rotation=label_angle,
                     color="black", bbox=dict(boxstyle='square', fc='white', ec='none', pad=0.2))
    ax.set_yticks([])


if __name__ == "__main__":
    # parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("file", nargs="?", default="display_config.yaml", help="Display config file (YAML).")
    args = parser.parse_args()
    # read and process config file, applying defaults
    with open(args.file) as f:
        config = yaml.load(f, Loader=yaml.Loader)
    default_config = config.pop("default", {})
    for display in config:
        config[display] = {**default_config, **config[display]}
        config[display]["display_size"] = tuple([int(x) for x in config[display]["display_size"].split('x')])
        config[display]["resolution"] = tuple([int(x) for x in config[display]["resolution"].split('x')])
    # generate plots
    plot_displays(config)
    plt.tight_layout()
    plt.show()
    plot_fov(config)
    plt.show()
