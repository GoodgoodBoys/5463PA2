from operator import truediv

import matplotlib.pyplot as plt
import numpy as np
import math
from utils.angle import angle_mod
from scipy.interpolate import make_interp_spline

show_animation = True

fig = plt.figure()
point = []

if show_animation:
    plt.ion()

def click(event):
    global x, y
    x = event.xdata
    y = event.ydata
    point.append([x, y])
    plt.plot(x, y, 'ro')
    fig.canvas.draw()

    if len(point) == 5:
        t = np.arange(len(point))
        x_vals, y_vals = zip(*point)

        tnew = np.linspace(t[0], t[-1], 200)

        spl_x = make_interp_spline(t, x_vals, k=3)
        spl_y = make_interp_spline(t, y_vals, k=3)

        xnew = spl_x(tnew)
        ynew = spl_y(tnew)

        plt.plot(xnew, ynew, 'g-', linewidth=2)

        fig.canvas.draw()


def main():  # pragma: no cover
    fig.canvas.mpl_connect("button_press_event", click)
    # fig.canvas.mpl_connect("button_release_event", click_done)

    plt.cla()
    plt.axis([-1.5, 1.5, -1.5, 1.5])

    circle_out = plt.Circle((0, 0), 1.5, color='lightgray', fill=False, linestyle='-')
    plt.gca().add_patch(circle_out)

    circle_inner = plt.Circle((0, 0), 1.0, color='lightgray', fill=False, linestyle='-')
    plt.gca().add_patch(circle_inner)

    plt.axis('equal')


    plt.show()
    plt.pause(5)


if __name__ == "__main__":
    # animation()
    main()
























