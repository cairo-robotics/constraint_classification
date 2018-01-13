#include "ros/ros.h"
#include "predicate_classification_msgs/PredicateClassification.h"
#include "predicate_classification/predicate_classifiers.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose.h"

bool run_classifier(predicate_class ification_msgs::PredicateClassification::Request  &req, predicate_classification_msgs::PredicateClassification::Response &res)
{
  typedef bool (*pfunc)(geometry_msgs::Pose, geometry_msgs::Pose, float);

  pfunc upright_classifier = &upright;
  pfunc proximity_classifier = &proximity;

  std::map <std::string, pfunc> predicate_classifier_map;
  predicate_classifier_map["upright"] = upright_classifier;
  predicate_classifier_map["proximity"] = proximity_classifier;
  
  res.upright_classification = predicate_classifier_map["upright"](req.upright.upright_pose, req.upright.current_pose, req.upright.threshold_angle);
  res.proximity_classification = predicate_classifier_map["proximity"](req.proximity.object_one_pose, req.proximity.object_two_pose, req.proximity.threshold_distance);

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