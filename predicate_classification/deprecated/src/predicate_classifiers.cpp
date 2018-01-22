#include "predicate_classification/predicate_classifiers.h"

namespace predicate_classifiers
{
  float getEuclideanDistance(float x1, float y1, float z1, float x2, float y2, float z2)
  {
    float delta_x = x1 - x2;
    float delta_y = y1 - y2;
    float delta_z = z1 - z2;
    float dist;

    dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
    return dist;
  }

  bool upright(geometry_msgs::Pose upright_pose, geometry_msgs::Pose current_pose, float threshold_angle)
  {
    // The upright quaternion orientation values.
    float upright_x = upright_pose.orientation.x;
    float upright_y = upright_pose.orientation.y;
    float upright_z = upright_pose.orientation.z;
    float upright_w = upright_pose.orientation.w; 

    // The current quaternion orientation values.
    float current_x = current_pose.orientation.x;
    float current_y = current_pose.orientation.y;
    float current_z = current_pose.orientation.z;
    float current_w = current_pose.orientation.w; 

    // We must transform a reference vector by the upright quaternion and the current quaternion and get the angle between the resulting vectors.
    // The angle between quaternions can be affected by yaw and therfore we cannot use angle difference methods of thee tf2::Quaternion class.
    tf2::Vector3 ref_vec = tf2::Vector3(0,0,1);
    tf2::Quaternion upright_q = tf2::Quaternion(upright_x, upright_y, upright_z, upright_w);
    tf2::Transform upright_tf = tf2::Transform(upright_q);
    tf2::Quaternion current_q = tf2::Quaternion(current_x, current_y, current_z, current_w);
    tf2::Transform current_tf = tf2::Transform(current_q);
    tf2::Vector3 upright_vec = upright_tf(ref_vec);
    tf2::Vector3 current_vec = current_tf(ref_vec);

    float angle = current_vec.angle(upright_vec) * 180/M_PI;

    ROS_INFO("Angle of deviation from upright to current: %f", angle);
    
    // Classification 
    if (angle < threshold_angle) {
      return 1;
    } else {
      return 0;
    }
  }

  bool proximity(geometry_msgs::Pose object_one_pose, geometry_msgs::Pose object_two_pose, float threshold_distance)
  {
    // The first object's pose position values.
    float object_one_pose_x = object_one_pose.position.x;
    float object_one_pose_y = object_one_pose.position.y;
    float object_one_pose_z = object_one_pose.position.z;

    // The second object's pose position values.
    float object_two_pose_x = object_two_pose.position.x;
    float object_two_pose_y = object_two_pose.position.y;
    float object_two_pose_z = object_two_pose.position.z;

    // Euclidean distance
    float distance = getEuclideanDistance(object_one_pose_x, object_one_pose_y, object_one_pose_z, object_two_pose_x, object_two_pose_y, object_two_pose_z);

    ROS_INFO("Distance between objects: %f", distance);

    // Check against the threshold distance that signifies the miniimum distance for two objects to be in proximity to eachother.
    if (distance < threshold_distance) {
      return 1;
    } else {
      return 0;
    }
  }

  bool height(geometry_msgs::Pose object_pose, float reference_height, float threshold_distance)
  {
    float object_height = object_pose.position.z;

    float difference = object_height - reference_height;

    ROS_INFO("Height differential: %f", difference);

    if (difference <= threshold_distance) {
      return 1;
    } else {
      return 0;
    }
  }
}





