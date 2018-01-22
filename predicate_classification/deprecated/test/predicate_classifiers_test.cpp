#include "predicate_classification_msgs/PredicateClassification.h"
#include "predicate_classification/predicate_classifiers.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose.h"
#include <gtest/gtest.h>


TEST(GetEuclidieanDistance, zeroEuclideanDistance)
{ 
  float actual_1 = 0;
  float test_1 = predicate_classifiers::getEuclideanDistance(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

  EXPECT_EQ(actual_1, test_1);
}

TEST(GetEuclidieanDistance, euclideanDistance)
{ 
  float actual_1 = 1.7320508;
  float test_1 = predicate_classifiers::getEuclideanDistance(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

  EXPECT_EQ(actual_1, test_1);
}

TEST(Upright, upsideDownTest)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;
  float threshold_angle = 25.0;
  
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = 0.0;
  upright_pose.orientation.y = 0.0;
  upright_pose.orientation.z = 0.0;
  upright_pose.orientation.w = 1.0;

  // R: 0, P: 175, Y: 0
  current_pose.position.x = 1.0;
  current_pose.position.y = .25;
  current_pose.position.z = .25;
  current_pose.orientation.x = 0.0;
  current_pose.orientation.y = 0.9990483518732844;
  current_pose.orientation.z = 0.0;
  current_pose.orientation.w = -0.043616403098766035;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(0, result);
}

TEST(Upright, uprightUsingPitchAndRollTest)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;

  float threshold_angle = 25.0;
  
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = 0.0;
  upright_pose.orientation.y = 0.0;
  upright_pose.orientation.z = 0.0;
  upright_pose.orientation.w = 1.0;

  // R: 10, P: 10, Y: 0
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.x = -0.08683579867169378;
  current_pose.orientation.y = -0.0868357986716937;
  current_pose.orientation.z = 0.007582702465964556;
  current_pose.orientation.w = 0.9924019300472017;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(1, result);
}

TEST(Upright, notUprightUsingPitchAndRollTest)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;

  float threshold_angle = 25.0;
  
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = 0.0;
  upright_pose.orientation.y = 0.0;
  upright_pose.orientation.z = 0.0;
  upright_pose.orientation.w = 1.0;

  // R: 35, P:-15, Y: 0
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.x = -0.2981390886391489;
  current_pose.orientation.y = 0.12450664849252854;
  current_pose.orientation.z = -0.03926361711858248;
  current_pose.orientation.w = 0.9455525086833311;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(0, result);
}

TEST(Upright, yawHasNoEffectTest)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;

  float threshold_angle = 5.0;
  
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = 0.0;
  upright_pose.orientation.y = 0.0;
  upright_pose.orientation.z = 0.0;
  upright_pose.orientation.w = 1.0;

  // R: 0, P: 0, Y: 20
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.x = 0;
  current_pose.orientation.y = 0;
  current_pose.orientation.z = -0.17367092187156372;
  current_pose.orientation.w = 0.9848037423244701;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(1, result);
}

TEST(Upright, angledUprightPitchOnlyTest)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;

  float threshold_angle = 26.0;
  
  // R: 0, P: 20, Y: 0
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = 0;
  upright_pose.orientation.y = -0.17367092187156372;
  upright_pose.orientation.z = 0;
  upright_pose.orientation.w = 0.9848037423244701;

  // R: 0, P: 45, Y: 0
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.x = 0;
  current_pose.orientation.y = -0.38270469415797664;
  current_pose.orientation.z =0;
  current_pose.orientation.w = 0.9238707253016787;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(1, result);
}

TEST(Upright, angleUprightNoRotation)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;

  float threshold_angle = 20;
  
  // R: 0, P: 20, Y: 0
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = 0.0;
  upright_pose.orientation.y = 0.1736709;
  upright_pose.orientation.z = 0.0;
  upright_pose.orientation.w = 0.9848037;

  // R: 0, P: 45, Y: 0
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.x = 0.0;
  current_pose.orientation.y = -0.38270469415797664;
  current_pose.orientation.z = 0.0;
  current_pose.orientation.w = 0.9238707253016787;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(1, result);
}


TEST(Upright, angledUprightUsingRollPitchYawTest)
{ 
  geometry_msgs::Pose upright_pose;
  geometry_msgs::Pose current_pose;

  float threshold_angle = 10.0;
  
  // R: 20, P: 10, Y: 10
  upright_pose.position.x = 0.0;
  upright_pose.position.y = 0.0;
  upright_pose.position.z = 0.0;
  upright_pose.orientation.x = -0.07043066500709379;
  upright_pose.orientation.y = -0.10057509181462758;
  upright_pose.orientation.z = 0.16484914083814878;
  upright_pose.orientation.w = 0.9786464801439086;

  // R: 20, P: 20, Y: 20
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.x = -0.13870300568743923;
  current_pose.orientation.y = -0.19810996120146637;
  current_pose.orientation.z = -0.13870300568743923;
  current_pose.orientation.w = 0.9603517041684749;

  int result = predicate_classifiers::upright(upright_pose, current_pose, threshold_angle);

  ASSERT_EQ(0, result);
}

TEST(Proximity, zeroDistanceProxmityTest)
{ 
  geometry_msgs::Pose obj_1;
  geometry_msgs::Pose obj_2;

  float threshold_distance = 1.0;
  
  obj_1.position.x = 0.0;
  obj_1.position.y = 0.0;
  obj_1.position.z = 0.0;
  obj_1.orientation.x = -0.07043066500709379;
  obj_1.orientation.y = -0.10057509181462758;
  obj_1.orientation.z = 0.16484914083814878;
  obj_1.orientation.w = 0.9786464801439086;

  obj_2.position.x = 0.0;
  obj_2.position.y = 0.0;
  obj_2.position.z = 0.0;
  obj_2.orientation.x = -0.13870300568743923;
  obj_2.orientation.y = -0.19810996120146637;
  obj_2.orientation.z = -0.13870300568743923;
  obj_2.orientation.w = 0.9603517041684749;

  int result = predicate_classifiers::proximity(obj_1, obj_2, threshold_distance);

  ASSERT_EQ(1, result);
}

TEST(Proximity, trueProximity)
{ 
  geometry_msgs::Pose obj_1;
  geometry_msgs::Pose obj_2;

  float threshold_distance = 3.0;
  
  obj_1.position.x = 0.0;
  obj_1.position.y = 0.0;
  obj_1.position.z = 0.0;
  obj_1.orientation.x = -0.07043066500709379;
  obj_1.orientation.y = -0.10057509181462758;
  obj_1.orientation.z = 0.16484914083814878;
  obj_1.orientation.w = 0.9786464801439086;

  obj_2.position.x = 2.0;
  obj_2.position.y = 0.0;
  obj_2.position.z = 0.0;
  obj_2.orientation.x = -0.13870300568743923;
  obj_2.orientation.y = -0.19810996120146637;
  obj_2.orientation.z = -0.13870300568743923;
  obj_2.orientation.w = 0.9603517041684749;

  int result = predicate_classifiers::proximity(obj_1, obj_2, threshold_distance);

  ASSERT_EQ(1, result);
}

TEST(Proximity, falseProximity)
{ 
  geometry_msgs::Pose obj_1;
  geometry_msgs::Pose obj_2;

  float threshold_distance = 1.0;
  
  obj_1.position.x = 0.0;
  obj_1.position.y = 0.0;
  obj_1.position.z = 0.0;
  obj_1.orientation.x = -0.07043066500709379;
  obj_1.orientation.y = -0.10057509181462758;
  obj_1.orientation.z = 0.16484914083814878;
  obj_1.orientation.w = 0.9786464801439086;

  obj_2.position.x = 2.0;
  obj_2.position.y = 0.0;
  obj_2.position.z = 0.0;
  obj_2.orientation.x = -0.13870300568743923;
  obj_2.orientation.y = -0.19810996120146637;
  obj_2.orientation.z = -0.13870300568743923;
  obj_2.orientation.w = 0.9603517041684749;

  int result = predicate_classifiers::proximity(obj_1, obj_2, threshold_distance);

  ASSERT_EQ(0, result);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
