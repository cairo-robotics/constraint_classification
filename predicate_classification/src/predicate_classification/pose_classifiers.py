import rospy
import numpy as np
from pyquaternion import Quaternion
from math import fabs
from predicate_classification.utils import angle_between


def upright(upright_pose, current_pose, threshold_angle, axis="z"):
    """
    Determines whether or not a current_pose is upright given the parameterizations.

    Parameters
    ----------
    upright_pose : geometry_msgs/Pose
        The upright pose.
    current_pose : geometry_msgs/Pose
        Current pose.
    threshold_angle : float/int
        Threshold angle within which the angle of deviation indicates the pose is upright.
    axis : str
        Axis against which to measure deviation.

    Returns
    -------
    : int
        1 if within threshold angle (upright), 0 otherwise.
    """
    if axis == "x":
        ref_vec = np.array([1., 0., 0.])  # Unit vector in the +x direction
    elif axis == "y":
        ref_vec = np.array([0., 1., 0.])  # Unit vector in the +y direction
    else:
        ref_vec = np.array([0., 0., 1.])  # Unit vector in the +z direction
    upright_q = Quaternion(upright_pose.orientation.w,
                           upright_pose.orientation.x,
                           upright_pose.orientation.y,
                           upright_pose.orientation.z)

    current_q = Quaternion(current_pose.orientation.w,
                           current_pose.orientation.x,
                           current_pose.orientation.y,
                           current_pose.orientation.z)

    upright_vec = upright_q.rotate(ref_vec)
    current_vec = current_q.rotate(ref_vec)
    angle = np.rad2deg(angle_between(upright_vec, current_vec))

    rospy.logdebug(
        "Angle of deviation from upright pose to current pose: {}".format(angle))

    # Classification
    if angle < threshold_angle:
        return 1
    else:
        return 0


def over_under(above_pose, below_pose, threshold_distance, axis="z"):
    """
    Determines whether one pose is above another pose given a threshold distance of deviation.
    The threshold distance of deviation means the radial distance around the vertical axis that
    determines if the classifier will register as true of false. The above object must have a greater
    positional value for the given axis dimension.

    Parameters
    ----------
    above_pose : geometry_msgs/Pose
        The above pose.
    below_pose : geometry_msgs/Pose
        The below pose.
    threshold_distance : float/int
        Threshold distance within which the poses are close enough to be over/under
    axis : str
        Vertical axis.

    Returns
    -------
    : int
        1 if above_pose is above the below_pose within a radial distance, 0 otherwise.
    """
    o1_x = above_pose.position.x
    o1_y = above_pose.position.y
    o1_z = above_pose.position.z

    o2_x = below_pose.position.x
    o2_y = below_pose.position.y
    o2_z = below_pose.position.z

    if axis == "x":
        if o1_x > o2_x:
            distance = np.linalg.norm(
                np.array([o1_y, o1_z]) - np.array([o2_y, o2_z]))
        else:
            return 0
    if axis == "y":
        if o1_y > o2_y:
            distance = np.linalg.norm(
                np.array([o1_y, o1_z]) - np.array([o2_y, o2_z]))
        else:
            return 0
    if axis == "z":
        if o1_z > o2_z:
            distance = np.linalg.norm(
                np.array([o1_y, o1_z]) - np.array([o2_y, o2_z]))
        else:
            return 0

    rospy.loginfo("Distance between objects: {}".format(distance))
    print(distance)
    if distance < threshold_distance:
        return 1
    else:
        return 0


def proximity(object1_pose, object2_pose, threshold_distance):
    """
    Determines whether or not are in proximity with each other.

    Parameters
    ----------
    object1_pose : geometry_msgs/Pose
        The upright pose.
    object2_pose : geometry_msgs/Pose
        Current pose.
    threshold_distance : float/int
        Threshold distance within two objects are in proximity.

    Returns
    -------
    : int
        1 if within distance (in proximity), 0 otherwise.
    """
    o1_x = object1_pose.position.x
    o1_y = object1_pose.position.y
    o1_z = object1_pose.position.z

    o2_x = object2_pose.position.x
    o2_y = object2_pose.position.y
    o2_z = object2_pose.position.z

    distance = np.linalg.norm(
        np.array([o1_x, o1_y, o1_z]) - np.array([o2_x, o2_y, o2_z]))

    rospy.logdebug("Distance between objects: {}".format(distance))

    if distance < threshold_distance:
        return 1
    else:
        return 0


def height(object_pose, reference_height, threshold_distance, direction="positive", axis="z"):
    """
    Determines whether or an object's pose is above a reference height.

    Parameters
    ----------
    object_pose : geometry_msgs/Pose
        The object's pose.
    reference_height : float/int
        The base axis value from which to measure the object's height. Using a negative reference value will indicate testing for below a height.
     threshold_distance : float/int
        Threshold distance within two objects are in proximity.
    axis : str
        Axis against which to measure deviation.

    Returns
    -------
    : int
        1 if above/below threshold distance, 0 otherwise.
    """
    if axis == "x":
        object_height = object_pose.position.x
    if axis == "y":
        object_height = object_pose.position.y
    if axis == "z":
        object_height = object_pose.position.z

    difference = fabs(object_height - reference_height)
    rospy.logdebug("Height differential: {}".format(difference))
    if direction == "positive":
        if object_height >= reference_height and difference >= threshold_distance:
            return 1
        else:
            return 0
    elif direction == "negative":
        if object_height <= reference_height and difference >= threshold_distance:
            return 1
        else:
            return 0
    else:
        raise ValueError("'direction' parameter must either be 'positive' or 'negative'")
    


