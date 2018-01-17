#include "ros/ros.h"
#include "predicate_classification_msgs/PredicateClassification.h"
#include "predicate_classification/predicate_classifiers.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib>
#include <gtest/gtest.h>

TEST(PredicateClassficationServiceTest, falseClassifications)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<predicate_classification_msgs::PredicateClassification>("predicate_classification");
  
  predicate_classification_msgs::PredicateClassification srv;

  srv.request.upright.upright_pose.position.x = 0.0;
  srv.request.upright.upright_pose.position.y = 0.0;
  srv.request.upright.upright_pose.position.z = 0.0;
  srv.request.upright.upright_pose.orientation.x = 0.0;
  srv.request.upright.upright_pose.orientation.y = 0.0;
  srv.request.upright.upright_pose.orientation.z = 0.0;
  srv.request.upright.upright_pose.orientation.w = 1.0;

  // Greater than 15 degree deviation
  srv.request.upright.current_pose.position.x = 0.0;
  srv.request.upright.current_pose.position.y = 0.0;
  srv.request.upright.current_pose.position.z = 0.0;
  srv.request.upright.current_pose.orientation.x = 0.0;
  srv.request.upright.current_pose.orientation.y = 0.2164559;
  srv.request.upright.current_pose.orientation.z = 0.0;
  srv.request.upright.current_pose.orientation.w = 0.9762924;

  srv.request.upright.threshold_angle = 15.0;

  // Greater that 1 distance.
  srv.request.proximity.object_one_pose.position.x = 0.0;
  srv.request.proximity.object_one_pose.position.y = 0.0;
  srv.request.proximity.object_one_pose.position.z = 0.0;
  srv.request.proximity.object_one_pose.orientation.x = 0.0;
  srv.request.proximity.object_one_pose.orientation.y = 0.0;
  srv.request.proximity.object_one_pose.orientation.z = 0.0;
  srv.request.proximity.object_one_pose.orientation.w = 1.0;

  srv.request.proximity.object_two_pose.position.x = 1.0;
  srv.request.proximity.object_two_pose.position.y = .25;
  srv.request.proximity.object_two_pose.position.z = .25;
  srv.request.proximity.object_two_pose.orientation.x = 0.0;
  srv.request.proximity.object_two_pose.orientation.y = 0.08715698831135904;
  srv.request.proximity.object_two_pose.orientation.z = 0.0;
  srv.request.proximity.object_two_pose.orientation.w = 0.9961945891182574;

  srv.request.proximity.threshold_distance = .5;

  client.call(srv);

  ASSERT_EQ(0, srv.response.upright_classification);
  ASSERT_EQ(0, srv.response.proximity_classification);
}

TEST(PredicateClassficationServiceTest, trueClassifications)
{ 
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<predicate_classification_msgs::PredicateClassification>("predicate_classification");
  
  predicate_classification_msgs::PredicateClassification srv;

  srv.request.upright.upright_pose.position.x = 0.0;
  srv.request.upright.upright_pose.position.y = 0.0;
  srv.request.upright.upright_pose.position.z = 0.0;
  srv.request.upright.upright_pose.orientation.x = 0.0;
  srv.request.upright.upright_pose.orientation.y = 0.0;
  srv.request.upright.upright_pose.orientation.z = 0.0;
  srv.request.upright.upright_pose.orientation.w = 1.0;

  // Less than 15 degree deviation 0.0454502, 0.087297, 0.2946111, 0.9505356
  srv.request.upright.current_pose.position.x = 0.0;
  srv.request.upright.current_pose.position.y = 0.0;
  srv.request.upright.current_pose.position.z = 0.0;
  srv.request.upright.current_pose.orientation.x = -0.06959150687262644;
  srv.request.upright.current_pose.orientation.y = -0.06959150687262644;
  srv.request.upright.current_pose.orientation.z = 0.004848794522172167;
  srv.request.upright.current_pose.orientation.w = 0.9951334249908784;

  srv.request.upright.threshold_angle = 15.0;

  // Less than 1 distance.
  srv.request.proximity.object_one_pose.position.x = 0.0;
  srv.request.proximity.object_one_pose.position.y = 0.0;
  srv.request.proximity.object_one_pose.position.z = 0.0;
  srv.request.proximity.object_one_pose.orientation.x = 0.0;
  srv.request.proximity.object_one_pose.orientation.y = 0.0;
  srv.request.proximity.object_one_pose.orientation.z = 0.0;
  srv.request.proximity.object_one_pose.orientation.w = 1.0;

  srv.request.proximity.object_two_pose.position.x = .25;
  srv.request.proximity.object_two_pose.position.y = .25;
  srv.request.proximity.object_two_pose.position.z = .25;
  srv.request.proximity.object_two_pose.orientation.x = 0.0;
  srv.request.proximity.object_two_pose.orientation.y = 0.08715698831135904;
  srv.request.proximity.object_two_pose.orientation.z = 0.0;
  srv.request.proximity.object_two_pose.orientation.w = 0.9961945891182574;

  srv.request.proximity.threshold_distance = 1.0;

  client.call(srv);

  ASSERT_EQ(1, srv.response.upright_classification);
  ASSERT_EQ(1, srv.response.proximity_classification);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){

  ros::init(argc, argv, "predicate_classification_client");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
