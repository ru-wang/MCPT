import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def UniformTriangle(n: int, vs):
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    pts = np.outer(vs[:, 0], 1 - np.sqrt(u1)) + np.outer(vs[:, 1], np.sqrt(u1) * u2)
    return pts


def main():
    COUNT = 3000

    mpl.use("Qt5Agg")

    fig, ax = plt.subplots()
    fig.tight_layout()

    # plot triangle
    pts = np.random.rand(2, 3)

    # plot uniform triangle samples
    xs, ys = UniformTriangle(COUNT, pts)
    ax.scatter(xs, ys, marker=".")

    # show plot
    ax.set(xlabel="X", ylabel="Y", aspect="equal")
    plt.show()


if __name__ == "__main__":
    main()
