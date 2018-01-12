cmake_minimum_required(VERSION 2.8.3)
project(predicate_classification)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  message_generation
  std_msgs
  geometry_msgs
  predicate_classification_msgs
  genmsg
  rostest
)

##########################
## CATKIN CONFIGURATION ##
##########################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs predicate_classification_msgs
 )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(predicate_classifiers ${PROJECT_SOURCE_DIR}/src/predicate_classifiers.cpp)
add_executable(predicate_classification_service ${PROJECT_SOURCE_DIR}/src/predicate_classification_service.cpp)
add_dependencies(predicate_classification_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(predicate_classifiers ${catkin_LIBRARIES})
target_link_libraries(predicate_classification_service ${catkin_LIBRARIES} predicate_classifiers)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(predicate_classification_test ${PROJECT_SOURCE_DIR}/test/predicate_classification.test ${PROJECT_SOURCE_DIR}/test/predicate_classification_service_test.cpp)
  add_rostest_gtest(predicate_classifiers_test ${PROJECT_SOURCE_DIR}/test/predicate_classifiers.test ${PROJECT_SOURCE_DIR}/test/predicate_classifiers_test.cpp)
  add_dependencies(predicate_classification_test ${catkin_EXPORTED_TARGETS})
  add_dependencies(predicate_classifiers_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(predicate_classification_test ${catkin_LIBRARIES} predicate_classifiers)
  target_link_libraries(predicate_classifiers_test ${catkin_LIBRARIES} predicate_classifiers)
endif()