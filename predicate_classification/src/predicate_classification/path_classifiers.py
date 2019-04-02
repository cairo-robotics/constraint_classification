from predicate_classification.utils import pt_in_polygon


def perimeter(pose, inner_polygon, outer_polygon, axis="z"):
    """
    Given coordinates of closed 2D polygon with a hole in the center, determines whether set of 2D coordinates is within polygon boundaries

    Parameters
    ----------
    pose : geometry_msgs/Pose
        Pose object with 2D coordinates 
    inner_polygon : list
        List of tuples (x,y) specifiying inner boundary of polygon
    outer_polygon: list
        List of tuples (x,y) specifiying outer boundary of polygon
    axis : str
        Axis against which to measure deviation.

    Returns
    -------
    : int
        1 if pose is inside polygon, 0 otherwise.
    """
    if axis == "z":
        p1 = pose.position.x
        p2 = pose.position.y
    elif axis == "y":
        p1 = pose.position.x
        p2 = pose.position.z
    elif axis == "x":
        p1 = pose.position.y
        p2 = pose.position.z

    if (pt_in_polygon(p1, p2, outer_polygon) is True) and (pt_in_polygon(p1, p2, inner_polygon) is False):
        return 1
    else:
        return 0
