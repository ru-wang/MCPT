import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def FindPlaneCoordinateSystem(normal: np.ndarray):
    assert normal.shape == (3,)

    z = normal.argmax()
    x = 0 if z == 2 else z + 1

    unit_x = np.zeros(3)
    unit_x[x] = 1

    axes = np.empty([3, 3])
    axes[:, 2] = normal
    axes[:, 1] = np.cross(axes[:, 2], unit_x)
    axes[:, 0] = np.cross(axes[:, 1], axes[:, 2])
    row_norms = np.sqrt((axes * axes).sum(axis=0))
    axes = axes / row_norms
    return axes


def Check(axes: np.ndarray):
    assert np.isclose(np.linalg.norm(axes[:, 0]), 1.0)
    assert np.isclose(np.linalg.norm(axes[:, 1]), 1.0)
    assert np.isclose(np.linalg.norm(axes[:, 2]), 1.0)

    assert np.allclose(np.cross(axes[:, 0], axes[:, 1]), axes[:, 2])
    assert np.allclose(np.cross(axes[:, 1], axes[:, 2]), axes[:, 0])
    assert np.allclose(np.cross(axes[:, 2], axes[:, 0]), axes[:, 1])


def main():
    Check(FindPlaneCoordinateSystem(np.array([0, 1, 10])))
    Check(FindPlaneCoordinateSystem(np.array([10, 1, 0])))
    Check(FindPlaneCoordinateSystem(np.array([1, 10, 0])))
    Check(FindPlaneCoordinateSystem(np.array([10, 10, 10])))


if __name__ == "__main__":
    main()
