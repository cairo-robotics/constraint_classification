cmake_minimum_required(VERSION 2.8.3)
project(predicate_classification)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  geometry_msgs
  predicate_classification_msgs
  genmsg
  rostest
)

catkin_python_setup()

##########################
## CATKIN CONFIGURATION ##
##########################

catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs predicate_classification_msgs
 )

###########
## Build ##
###########

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(src/test/predicate_classification_server.test)
  add_rostest(src/test/pose_classifiers.test)
endif()