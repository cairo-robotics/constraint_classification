#!/usr/bin/env python2

import rospy
import argparse
from predicate_classification.predicate_classifiers import upright
from geometry_msgs.msg import Pose

arg_fmt = argparse.RawDescriptionHelpFormatter
parser = argparse.ArgumentParser(formatter_class=arg_fmt)
required = parser.add_argument_group('required arguments')
parser.add_argument('--upright', nargs=4, help='An object\'s upright quaternion values: --upright x y z w')
parser.add_argument('--current', nargs=4, help='An objects\'s current quaternion values: --upright x y z w')
args = parser.parse_args(rospy.myargv()[1:])

upright_args = [float(i) for i in args.upright]
current_args = [float(i) for i in args.current]

upright_pose = Pose()
upright_pose.position.x = 0.0
upright_pose.position.y = 0.0
upright_pose.position.z = 0.0
upright_pose.orientation.x = upright_args[0]
upright_pose.orientation.y = upright_args[1]
upright_pose.orientation.z = upright_args[2]
upright_pose.orientation.w = upright_args[3]

current_pose = Pose()
current_pose.position.x = 0.0
current_pose.position.y = 0.0
current_pose.position.z = 0.0
current_pose.orientation.x = current_args[0]
current_pose.orientation.y = current_args[1]
current_pose.orientation.z = current_args[2]
current_pose.orientation.w = current_args[3]

print upright(upright_pose, current_pose, 20, axis = "x")


# Axis = 'x'
# Level with buttons facing up.
# 0.712590112587 -0.00994445446764 0.701496927312 -0.00430119065513
# Button cuff facing 90 degrees to one side
# -0.497165522925 0.508076415174 -0.499403654355 0.495258305509
# Result: ~90 degrees :)

# Axis = 'x'
# Level with buttons facing up.
# 0.712590112587 -0.00994445446764 0.701496927312 -0.00430119065513
# Button cuff facing 90 degrees to forward
# 0.999480156993 -0.00346147168356 -0.0156511927595 -0.0279727395153
# Result: ~90 degrees :)


# Axis = 'x'
# Level with buttons facing up.
# 0.712590112587 -0.00994445446764 0.701496927312 -0.00430119065513
# Button cuff facing ~45 degrees to diagonally from level
# 0.888835870709 -0.277907278582 0.349356863595 0.103383370571
# Result: ~45 degrees :)


# Axis = 'x'
# Level with buttons facing up.
# 0.712590112587 -0.00994445446764 0.701496927312 -0.00430119065513
# Button cuff facing ~45 degrees to diagonally from level on opposite side of robot
# 0.826073892364 0.427656736112 0.294250334593 -0.219381815576
# Result: ~45 degrees :)
