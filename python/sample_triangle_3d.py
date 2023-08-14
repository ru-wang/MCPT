import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def UniformTriangle(n: int, vs):
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    pts = (
        np.outer(vs[:, 0], 1 - np.sqrt(u1))
        + np.outer(vs[:, 1], np.sqrt(u1) * (1 - u2))
        + np.outer(vs[:, 2], np.sqrt(u1) * u2)
    )
    return pts


def main():
    assert len(sys.argv) == 2
    n = int(sys.argv[1])

    mpl.use("Qt5Agg")

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    fig.tight_layout()

    # plot triangle
    pts = np.random.rand(3, 3)
    ax.plot_trisurf(pts[0], pts[1], pts[2], alpha=0.1)

    # plot uniform triangle samples
    xs, ys, zs = UniformTriangle(n, pts)
    ax.scatter(xs, ys, zs, s=0.5, marker=".")

    # show plot
    ax.set(xlabel="X", ylabel="Y", zlabel="Z", aspect="equal")
    plt.show()


if __name__ == "__main__":
    main()
