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

# if(CATKIN_ENABLE_TESTING)
#   add_rostest_gtest(predicate_classification_test ${PROJECT_SOURCE_DIR}/test/predicate_classification.test ${PROJECT_SOURCE_DIR}/test/predicate_classification_service_test.cpp)
#   add_rostest_gtest(predicate_classifiers_test ${PROJECT_SOURCE_DIR}/test/predicate_classifiers.test ${PROJECT_SOURCE_DIR}/test/predicate_classifiers_test.cpp)
#   add_dependencies(predicate_classification_test ${catkin_EXPORTED_TARGETS})
#   add_dependencies(predicate_classifiers_test ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(predicate_classification_test ${catkin_LIBRARIES} predicate_classifiers)
#   target_link_libraries(predicate_classifiers_test ${catkin_LIBRARIES} predicate_classifiers)
# endif()