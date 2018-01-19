import math
import numpy as np
import roslib; roslib.load_manifest('tf2_geometry_msgs')
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped

 class UprightClassifier:

    def upright(upright_msg):
        upright_x = 
        upright_y = upright_msg.upright_pose.orientation.y
        upright_z = upright_msg.upright_pose.orientation.z
        upright_w = upright_msg.upright_pose.orientation.w 
        upright_q = [upright_x, upright_y, upright_z, upright_w]

        current_x = upright_msg.current_pose.orientation.x
        current_y = upright_msg.current_pose.orientation.y
        current_z = upright_msg.current_pose.orientation.z
        current_w = upright_msg.current_pose.orientation.w
        current_q = [current_x, current_y, current_z, current_w]
        

        # a translation should not modify a Vector3 (only rotation should)
        # testing with a transform that is a pure translation
        upright_t = TransformStamped()
        upright_t.transform.rotation.x = upright_msg.upright_pose.orientation.x
        upright_t.transform.rotation.y = upright_msg.upright_pose.orientation.y
        upright_t.transform.rotation.z = upright_msg.upright_pose.orientation.z
        upright_t.transform.rotation.w = upright_msg.upright_pose.orientation.w

        ref_v = Vector3Stamped()
        ref_v.vector.x = 0
        ref_v.vector.y = 0
        ref_v.vector.z = 1

        vt = tf2_geometry_msgs.do_transform_vector3(ref_v, t)
        test_similar('translation should not be applied', vt.vector, v.vector.x, v.vector.y, v.vector.z)


        t = TransformStamped()

        v = Vector3Stamped()
        v.vector.x = 0
        v.vector.y = 0
        v.vector.z = 1
