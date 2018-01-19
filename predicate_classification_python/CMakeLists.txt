cmake_minimum_required(VERSION 3.5)
project(predicate_classification_python)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	predicate_classification
	predicate_classification_msgs
)

find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

catkin_python_setup()

catkin_package(
    LIBRARIES ${predicate_classifiers_INCLUDE_DIRS}
	CATKIN_DEPENDS rospy roscpp message_runtime std_msgs geometry_msgs predicate_classification predicate_classification_msgs
)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

add_library(predicate_classifiers_py SHARED src/classifiers.cpp)

target_link_libraries(predicate_classifiers_py ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${predicate_classifiers_INCLUDE_DIRS})

set_target_properties(predicate_classifiers_py PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
 )