import numpy as np


def unit_vector(vector):
    """
    Calculates the unit vector of a vector.

    Parameters
    ----------
    vector : array-like
        Input vector.

    Returns
    -------
    : array-like
        The unit vector of the input vector.
    """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """
    Calculates the angle in radians between vectors.

    Parameters
    ----------
    v1 : array-like
        First vector.
    v2 : array-like
        Second vector.

    Returns
    -------
    : float
        Angle between v1 and v1 in radians.
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
