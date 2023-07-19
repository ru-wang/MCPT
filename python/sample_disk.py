import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def NonUniformDisk(n: int):
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    # [0, 1)
    r = u1
    # [0, 2pi)
    theta = 2.0 * np.pi * u2
    # convert polar coordinates
    xs = r * np.cos(theta)
    ys = r * np.sin(theta)
    return [xs, ys]


def UniformDisk(n: int):
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    # [0, 1)
    r = np.sqrt(u1)
    # [0, 2pi)
    theta = 2.0 * np.pi * u2
    # convert polar coordinates
    xs = r * np.cos(theta)
    ys = r * np.sin(theta)
    return [xs, ys]


def main():
    COUNT = 10000

    mpl.use("Qt5Agg")

    fig, ax = plt.subplots()
    fig.tight_layout()

    # plot circle
    circle = plt.Circle((0, 0), radius=1.0, color="red", alpha=0.1)
    ax.add_artist(circle)

    # plot non-uniform disk
    # xs, ys = NonUniformDisk(COUNT)
    # ax.scatter(xs, ys, c="green", marker=".", alpha=0.5)

    # plot uniform disk
    xs, ys = UniformDisk(COUNT)
    ax.scatter(xs, ys, c="blue", marker=".", alpha=0.5)

    # show plot
    ax.set(xticklabels=[], yticklabels=[], aspect="equal")
    plt.show()


if __name__ == "__main__":
    main()
