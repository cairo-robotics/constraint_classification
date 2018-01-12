#ifndef _PREDICATE_CLASSIFIERS_H_  // To make sure you don't declare the function more than once by including the header multiple times.
#define _PREDICATE_CLASSIFIERS_H_

#include "ros/ros.h"
#include "predicate_classification_msgs/PredicateClassification.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/Pose.h"
#include <math.h>
 
float getEuclideanDistance(float x1, float y1, float z1, float x2, float y2, float z2);

bool upright(geometry_msgs::Pose upright_pose, geometry_msgs::Pose current_pose, float threshold_angle);

bool proximity(geometry_msgs::Pose object_one_pose, geometry_msgs::Pose object_two_pose, float threshold_distance);

#endif