import rospy
import numpy as np
from pyquaternion import Quaternion
from math import pow, fabs


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'."""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def get_euclidean_distance(x1, y1, z1, x2, y2, z2):
    delta_x = x1 - x2
    delta_y = y1 - y2
    delta_z = z1 - z2

    return np.sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2))


def upright(upright_pose, current_pose, threshold_angle, axis="z"):

    if axis is "x":
        ref_vec = np.array([1., 0., 0.]) # Unit vector in the +x direction
    if axis is "y":
        ref_vec = np.array([0., 1., 0.]) # Unit vector in the +y direction
    if axis is "z":
        ref_vec = np.array([0., 0., 1.]) # Unit vector in the +z direction
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

    rospy.loginfo("Angle of devation from upright pose to current pose: {}".format(angle))
    print(angle)
    # Classification
    if angle < threshold_angle:
        return 1
    else:
        return 0


def proximity(object1_pose, object2_pose, threshold_distance):
    o1_x = object1_pose.position.x
    o1_y = object1_pose.position.y
    o1_z = object1_pose.position.z

    o2_x = object2_pose.position.x
    o2_y = object2_pose.position.y
    o2_z = object2_pose.position.z

    distance = get_euclidean_distance(o1_x, o1_y, o1_z, o2_x, o2_y, o2_z)

    rospy.loginfo("Distance between objects: {}".format(distance))

    if distance < threshold_distance:
        return 1
    else:
        return 0


def height(object_pose, reference_height, threshold_distance):
    object_height = object_pose.position.z

    difference = fabs(object_height - reference_height)

    rospy.loginfo("Height differential: {}".format(difference))

    if difference >= threshold_distance:
        return 1
    else:
        return 0

    