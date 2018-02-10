# predicate_classifcation
ROS service providing classification for binary valued predicates.

Currently supported predicates:

upright: Given an upright reference pose, the current pose, and threshold angle, determines if the object is upright.
proximity: Determines whether two objects are in proximity with eachother using euclidean distance.
height: Determines whether an object is above or below a reference height given a threshold distance from than reference.

## Running tests

Build the package

  your_ws/src/predicate_classification$ catkin build

Run the tests

  your_ws/src/predicate_classification$ catkin build --make-args run_tests

Change directory to your_ws/build/predicate_classification/test_results/predicate_classification to see the test results.



