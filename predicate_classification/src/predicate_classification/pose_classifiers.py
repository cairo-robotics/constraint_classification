import rospy
import numpy as np
from pyquaternion import Quaternion
from math import fabs
from predicate_classification.utils import angle_between, quaternion_to_euler


def cone(orientation_pose, current_pose, threshold_angle, axis):
    """
    Determines if an object's orientation within a cone centered around a given axis and with a threshold angle dead center in that cone. 

    Parameters
    ----------
    orientation_pose : geometry_msgs/Pose
        The correct orientation pose.
    current_pose : geometry_msgs/Pose
        Current pose.
    threshold_angle : float/int
        Threshold angle within which the angle of deviation indicates the pose is correct.

    Returns
    -------
    : int
        1 if within threshold angle, 0 otherwise.
    """

    if axis == "x":
        ref_vec = np.array([1., 0., 0.])  # Unit vector in the +x direction
    elif axis == "y":
        ref_vec = np.array([0., 1., 0.])  # Unit vector in the +y direction
    else:
        ref_vec = np.array([0., 0., 1.])  # Unit vector in the +z direction
    correct_q = Quaternion(orientation_pose.orientation.w,
                           orientation_pose.orientation.x,
                           orientation_pose.orientation.y,
                           orientation_pose.orientation.z)

    current_q = Quaternion(current_pose.orientation.w,
                           current_pose.orientation.x,
                           current_pose.orientation.y,
                           current_pose.orientation.z)
    correct_q_inv = correct_q.inverse if correct_q != Quaternion(0, 0, 0, 0) else correct_q
    q_prime = correct_q_inv * current_q

    rot_vec = q_prime.rotate(ref_vec)
    angle = np.rad2deg(angle_between(ref_vec, rot_vec))
    rospy.logdebug(
        "Angle of deviation from correct orientation pose to current pose: {}".format(angle))
    # Classification
    if angle < threshold_angle:
        return 1
    else:
        return 0


def twist(orientation_pose, current_pose, threshold_angle):
    """
    Determines whether or not a current_pose rolled beyond a threshold angle relative to the reference quaternion.

    Parameters
    ----------
    orientation_pose : geometry_msgs/Pose
        The correct orientation pose.
    current_pose : geometry_msgs/Pose
        Current pose.
    threshold_angle : float/int
        Threshold angle within which the angle of deviation indicates the pose is correct.

    Returns
    -------
    : int
        1 if within threshold angle, 0 otherwise.
    """
    ref_vec = [1.0, 1.0, 1.0]

    correct_q = Quaternion(orientation_pose.orientation.w,
                           orientation_pose.orientation.x,
                           orientation_pose.orientation.y,
                           orientation_pose.orientation.z)

    current_q = Quaternion(current_pose.orientation.w,
                           current_pose.orientation.x,
                           current_pose.orientation.y,
                           current_pose.orientation.z)
    correct_q_inv = correct_q.inverse if correct_q != Quaternion(0, 0, 0, 0) else correct_q
    q_prime = correct_q_inv * current_q

    q_prime_fixed = correct_q * q_prime * correct_q.inverse

    
    roll, pitch, yaw = quaternion_to_euler(q_prime_fixed[1], q_prime_fixed[2], q_prime_fixed[3], q_prime_fixed[0])
    
  
    twist_val = roll

    rospy.logdebug(
        "Twist/roll angle: {}".format(twist_val))
    
    # Classification
    if abs(twist_val) <= threshold_angle:
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
        # NO CHECK "ABOVE" SINCE THIS IS MORE OF A CENTERING CONSTRAINT
        distance = np.linalg.norm(
                np.array([o1_y, o1_z]) - np.array([o2_y, o2_z]))
        
    if axis == "y":
        # NO CHECK "ABOVE" SINCE THIS IS MORE OF A CENTERING CONSTRAINT
        distance = np.linalg.norm(
                np.array([o1_x, o1_z]) - np.array([o2_x, o2_z]))
    if axis == "z":
        distance = np.linalg.norm(
                np.array([o1_x, o1_y]) - np.array([o2_x, o2_y]))
  

    rospy.logdebug("Distance from centerline: {}".format(distance))

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


def planar(object_pose, reference_position, threshold_distance, direction="positive", axis="z"):
    """
    Determines whether or an object's pose positively or negatively distance away from a reference_position and threshold distance along the given value.

    Parameters
    ----------
    object_pose : geometry_msgs/Pose
        The object's pose.
    reference_position : float/int
        The reference or starting position to compare an objects height distance against the threshold_distance along the given axis.
     threshold_distance : float/int
        Threshold distance away from to evaluate. 
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

    difference = fabs(object_height - reference_position)
    if direction == "positive":
        if object_height >= reference_position and difference >= threshold_distance:
            return 1
        else:
            return 0
    elif direction == "negative":
        if object_height <= reference_position and difference >= threshold_distance:
            return 1
        else:
            return 0
    else:
        raise ValueError(
            "'direction' parameter must either be 'positive' or 'negative'")
