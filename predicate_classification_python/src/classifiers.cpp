#include <boost/python.hpp>
#include "ros/ros.h"
#include "predicate_classification_msgs/PredicateClassification.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "predicate_classification/predicate_classifiers.h"


BOOST_PYTHON_MODULE(predicate_classifiers){

	using namespace boost::python;

	class_<predicate_classifiers::ProximityClassifier>("ProximityClassifier")
		.def("getEuclideanDistance", &predicate_classifiers::ProximityClassifier::getEuclideanDistance, "bool getEuclideanDistance() returns the euclidean distance of a pair of points")
		.def("classify", &predicate_classifiers::ProximityClassifier::classify, "bool classify() returns the boolean result of whether or not a objects is close to another given a threshold.");
		
	class_<predicate_classifiers::UprightClassifier>("UprightClassifier")
		.def("classify", &predicate_classifiers::UprightClassifier::classify, "bool classify() returns the boolean result of whether or not a object's current pose is upright accorrding to angle of deviation and upright pose critera.");

	class_<predicate_classifiers::HeightClassifier>("HeightClassifier")
		.def("classify", &predicate_classifiers::HeightClassifier::classify, "bool classify() returns the boolean result of whether or not an objects is within a given threshold height according to the z-axis.");
}
