#!/usr/bin/env python

import sys
import rospy
from predicate_classification_msgs.srv import PredicateClassification
from predicate_classification_msgs.msg import Upright, Proximity, Height
from geometry_msgs.msg import Pose


def predicate_classification_client(upright, proximity, height):
    rospy.wait_for_service('predicate_classification_server')
    try:
        predicate_classifier = rospy.ServiceProxy('predicate_classification_server', PredicateClassification)
        response = predicate_classifier(upright, proximity, height)
        return response
    except rospy.ServiceException as e:
        print "Service call failed: {}".format(e)


if __name__ == "__main__":
    
    upright_msg = Upright()
    upright_pose = Pose()
    current_pose = Pose()
#0, 0.1736709, 0, 0.9848037 
    upright_pose.position.x = 0.0
    upright_pose.position.y = 0.0
    upright_pose.position.z = 0.0
    upright_pose.orientation.x = 0.0
    upright_pose.orientation.y = 0.0
    upright_pose.orientation.z = 0.0
    upright_pose.orientation.w = 1.0

    current_pose.position.x = 0.0
    current_pose.position.y = 0.0
    current_pose.position.z = 0.0
    current_pose.orientation.x = 0.0
    current_pose.orientation.y = 0.1736709
    current_pose.orientation.z = 0.0
    current_pose.orientation.w = 0.9848037 

    upright_msg.upright_pose = upright_pose
    upright_msg.current_pose = current_pose
    upright_msg.threshold_angle = 21.0

    proximity_msg = Proximity()
    object_one_pose = Pose()
    object_two_pose = Pose()

    object_one_pose.position.x = 0.0
    object_one_pose.position.y = 0.0
    object_one_pose.position.z = 0.0
    object_one_pose.orientation.x = 0.0
    object_one_pose.orientation.y = 0.0
    object_one_pose.orientation.z = 0.0
    object_one_pose.orientation.w = 1.0

    object_two_pose.position.x = 1.0
    object_two_pose.position.y = 0.25
    object_two_pose.position.z = 0.25
    object_two_pose.orientation.x = 0.0
    object_two_pose.orientation.y = 0.0
    object_two_pose.orientation.z = 0.0
    object_two_pose.orientation.w = 1.0

    proximity_msg.object_one_pose = object_one_pose
    proximity_msg.object_two_pose = object_two_pose
    proximity_msg.threshold_distance = 5.0

    height_msg = Height()
    object_pose = Pose()

    object_pose.position.x = 0.0
    object_pose.position.y = 0.0
    object_pose.position.z = 3.0
    object_pose.orientation.x = 0.0
    object_pose.orientation.y = 0.0
    object_pose.orientation.z = 0.0
    object_pose.orientation.w = 1.0

    height_msg.object_pose = object_pose
    height_msg.threshold_distance = 5.0
    height_msg.reference_height = 0.0

    print "Requesting {}".format(predicate_classification_client)
    print "Classification results:\n{}".format(predicate_classification_client(upright_msg, proximity_msg, height_msg))
