#include "ros/ros.h"
#include "predicate_classification_msgs/PredicateClassification.h"
#include "predicate_classification/predicate_classifiers.h"
#include "geometry_msgs/Pose.h"

bool run_classifier(predicate_classification_msgs::PredicateClassification::Request  &req, predicate_classification_msgs::PredicateClassification::Response &res)
{ 
  predicate_classifiers::ProximityClassifier proximity_classifer = predicate_classifiers::ProximityClassifier();
  predicate_classifiers::UprightClassifier upright_classifer = predicate_classifiers::UprightClassifier();
  predicate_classifiers::HeightClassifier height_classifer = predicate_classifiers::HeightClassifier();

  res.upright_classification = upright_classifer.classify(req.upright.upright_pose, req.upright.current_pose, req.upright.threshold_angle);
  res.proximity_classification = proximity_classifer.classify(req.proximity.object_one_pose, req.proximity.object_two_pose, req.proximity.threshold_distance);
  res.height_classification = height_classifer.classify(req.height.object_pose, req.height.reference_height, req.height.threshold_distance);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "predicate_classification_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("predicate_classification", run_classifier);
  ROS_INFO("Ready to perform predicate classification.");
  ros::spin();

  return 0;
}