import rospy
from predicate_classifiers import HeightClassifier, UprightClassifier, ProximityClassifer
from predicate_classification_msgs import PredicateClassification

def run_classifier(req):
    response = {

    }
    height

def predicate_classification_server():
    rospy.init_node('predicate_classification_server')
    s = rospy.Service('predicate_classification_server', PredicateClassification, run_classifier)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    predicate_classification_server()
