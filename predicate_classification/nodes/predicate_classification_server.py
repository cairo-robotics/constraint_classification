#!/usr/bin/env python

import rospy
from predicate_classification.predicate_classifiers import height, upright, proximity
from predicate_classification_msgs.srv import PredicateClassification


def run_classifier(req):
    response = {
        "upright_classification": upright(req.upright.upright_pose, req.upright.current_pose, req.upright.threshold_angle),
        "proximity_classification": proximity(req.proximity.object_one_pose, req.proximity.object_two_pose, req.proximity.threshold_distance),
        "height_classification": height(req.height.object_pose, req.height.reference_height, req.height.threshold_distance)
    }

    return response


def predicate_classification_server():
    rospy.init_node('predicate_classification_server')
    s = rospy.Service('predicate_classification_server', PredicateClassification, run_classifier)
    print "Predicate classification server running..."
    rospy.spin()

if __name__ == "__main__":
    predicate_classification_server()
