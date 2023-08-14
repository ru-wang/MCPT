import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def UniformTriangle(n: int, vs):
    s = np.random.rand(n)
    t = np.sqrt(np.random.rand(n))
    pts = (
        np.outer(vs[:, 0], t * (1 - s))
        + np.outer(vs[:, 1], (1 - t))
        + np.outer(vs[:, 2], s * t)
    )
    return pts


def main():
    assert len(sys.argv) == 2
    n = int(sys.argv[1])

    mpl.use("Qt5Agg")

    fig, ax = plt.subplots()
    fig.tight_layout()

    # plot triangle
    pts = np.random.rand(2, 3)
    ax.triplot(pts[0], pts[1])

    # plot uniform triangle samples
    xs, ys = UniformTriangle(n, pts)
    ax.scatter(xs, ys, s=0.5, marker=".")

    # show plot
    ax.set(xlabel="X", ylabel="Y", aspect="equal")
    plt.show()


if __name__ == "__main__":
    main()
