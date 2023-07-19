import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

TWO_PI = 2.0 * np.pi
HALF_PI = np.pi / 2.0


def UniformHemisphere(n: int):
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    # [0, 2pi)
    azi = 2.0 * np.pi * u1
    # [0, pi/2)
    dpr = np.arccos(1.0 - u2)
    # PDF = 1.0 / (2pi)
    return azi, dpr, 1.0 / TWO_PI


def CosineHemisphere(n: int):
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    # [0, 2pi)
    azi = 2.0 * np.pi * u1
    # [0, pi/2)
    dpr = np.arcsin(np.sqrt(u2))
    # PDF = cos(depression) / pi
    pdf = np.cos(dpr) / np.pi
    return azi, dpr, pdf


def CosinePowerHemisphere(n: int, alpha: float):
    alpha_1 = alpha + 1.0
    u1 = np.random.rand(n)
    u2 = np.random.rand(n)
    # [0, 2pi)
    azi = 2.0 * np.pi * u1
    # [0, pi/2)
    dpr = np.arccos(np.power(1.0 - u2, 1.0 / alpha_1))
    # PDF = (a+1)cos^a(depression)/(2pi)
    pdf = alpha_1 * np.power(np.cos(dpr), alpha) / TWO_PI
    return azi, dpr, pdf


def MakePoints(gen: callable, count: int, **kwargs):
    azi, dpr, pdf = gen(count, **kwargs)
    xs = np.multiply(np.sin(dpr), np.cos(azi))
    ys = np.multiply(np.sin(dpr), np.sin(azi))
    zs = np.cos(dpr)
    return xs, ys, zs, pdf


def MakeHemisphere(m: int, n: int):
    u = np.linspace(0.0, TWO_PI, m)
    v = np.linspace(0.0, HALF_PI, n)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones(m), np.cos(v))
    return xs, ys, zs


def main():
    COUNT = 3000

    mpl.use("Qt5Agg")

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    fig.tight_layout()

    # plot hemisphere wire frame
    xs, ys, zs = MakeHemisphere(30, 20)
    ax.plot_wireframe(xs, ys, zs, color=np.full(4, 0.25))

    # plot uniform hemisphere samples
    xs, ys, zs, pdf = MakePoints(UniformHemisphere, COUNT)
    ax.scatter(xs, ys, zs, marker=".", alpha=0.5)

    # plot cosine weighted hemisphere samples
    xs, ys, zs, pdf = MakePoints(CosineHemisphere, COUNT)
    ax.scatter(xs, ys, zs, marker=".", alpha=0.5)

    # plot cosine power weighted hemisphere samples
    xs, ys, zs, pdf = MakePoints(CosinePowerHemisphere, COUNT, alpha=5.0)
    ax.scatter(xs, ys, zs, marker=".", alpha=0.5)

    # show plot
    ax.set(xticklabels=[], yticklabels=[], zticklabels=[], aspect="equal")
    plt.show()


if __name__ == "__main__":
    main()
