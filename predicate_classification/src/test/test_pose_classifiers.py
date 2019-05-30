#!/usr/bin/env python

import sys
import unittest
from geometry_msgs.msg import Pose
from predicate_classification.pose_classifiers import height, upright, proximity, over_under


class TestUprightClassifier(unittest.TestCase):

    def test_upside_down(self):
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.0
        upright_pose.orientation.y = 0.0
        upright_pose.orientation.z = 0.0
        upright_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = 0.9990483518732844
        current_pose.orientation.z = 0.0
        current_pose.orientation.w = -0.043616403098766035

        threshold_angle = 25

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 0)

    def test_upright_pitch_and_roll(self):
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.0
        upright_pose.orientation.y = 0.0
        upright_pose.orientation.z = 0.0
        upright_pose.orientation.w = 1.0

        # R: 10, P: 10, Y: 0
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = -0.08683579867169378
        current_pose.orientation.y = -0.0868357986716937
        current_pose.orientation.z = 0.007582702465964556
        current_pose.orientation.w = 0.9924019300472017

        threshold_angle = 25

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 1)

    def test_not_upright_pitch_and_roll(self):
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.0
        upright_pose.orientation.y = 0.0
        upright_pose.orientation.z = 0.0
        upright_pose.orientation.w = 1.0

        # R: 35, P:-15, Y: 0
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = -0.2981390886391489
        current_pose.orientation.y = 0.12450664849252854
        current_pose.orientation.z = -0.03926361711858248
        current_pose.orientation.w = 0.9455525086833311

        threshold_angle = 25

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 0)

    def test_yaw_has_no_effect(self):
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.0
        upright_pose.orientation.y = 0.0
        upright_pose.orientation.z = 0.0
        upright_pose.orientation.w = 1.0

        # R: 0, P: 0, Y: 20
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = 0.0
        current_pose.orientation.z = -0.17367092187156372
        current_pose.orientation.w = 0.9848037423244701

        threshold_angle = 5

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 1)

    def test_angled_upright_pitch_only(self):
        # R: 0, P: 20, Y: 0
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.0
        upright_pose.orientation.y = -0.17367092187156372
        upright_pose.orientation.z = 0.0
        upright_pose.orientation.w = 0.9848037423244701

        # R: 0, P: 45, Y: 0
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = -0.38270469415797664
        current_pose.orientation.z = 0.0
        current_pose.orientation.w = 0.9238707253016787

        threshold_angle = 26

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 1)

    def test_angled_upright_no_rotation(self):
        # R: 0, P: 20, Y: 0
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.0
        upright_pose.orientation.y = 0.1736709
        upright_pose.orientation.z = 0.0
        upright_pose.orientation.w = 0.9848037

        # R: 0, P: 45, Y: 0
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = -0.38270469415797664
        current_pose.orientation.z = 0.0
        current_pose.orientation.w = 0.9238707253016787

        threshold_angle = 20

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 0)

    def test_angled_upright_with_roll_pitch_yaw(self):
        # R: 20, P: 10, Y: 10
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = -0.07043066500709379
        upright_pose.orientation.y = -0.10057509181462758
        upright_pose.orientation.z = 0.16484914083814878
        upright_pose.orientation.w = 0.9786464801439086

        # R: 20, P: 20, Y: 20
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = -0.13870300568743923
        current_pose.orientation.y = -0.19810996120146637
        current_pose.orientation.z = -0.13870300568743923
        current_pose.orientation.w = 0.9603517041684749

        threshold_angle = 10

        result = upright(upright_pose, current_pose, threshold_angle)
        self.assertEquals(result, 0)

    def test_x_axis_rotation_around_z(self):
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.712590112587
        upright_pose.orientation.y = -0.00994445446764
        upright_pose.orientation.z = 0.701496927312
        upright_pose.orientation.w = -0.00430119065513

        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = -0.497165522925
        current_pose.orientation.y = 0.508076415174
        current_pose.orientation.z = -0.499403654355
        current_pose.orientation.w = 0.495258305509

        threshold_angle = 95

        result = upright(upright_pose, current_pose, threshold_angle, axis="x")
        self.assertEquals(result, 1)

        threshold_angle = 85

        result = upright(upright_pose, current_pose, threshold_angle, axis="x")
        self.assertEquals(result, 0)

    def test_x_axis_diagonal_rotation(self):
        upright_pose = Pose()
        upright_pose.position.x = 0.0
        upright_pose.position.y = 0.0
        upright_pose.position.z = 0.0
        upright_pose.orientation.x = 0.712590112587
        upright_pose.orientation.y = -0.00994445446764
        upright_pose.orientation.z = 0.701496927312
        upright_pose.orientation.w = -0.00430119065513

        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.position.z = 0.0
        current_pose.orientation.x = 0.888835870709
        current_pose.orientation.y = -0.277907278582
        current_pose.orientation.z = 0.349356863595
        current_pose.orientation.w = 0.103383370571

        threshold_angle = 50

        result = upright(upright_pose, current_pose, threshold_angle, axis="x")
        self.assertEquals(result, 1)

        threshold_angle = 40

        result = upright(upright_pose, current_pose, threshold_angle, axis="x")
        self.assertEquals(result, 0)


class TestProximityClassifier(unittest.TestCase):

    def test_zero_distance(self):
        object_one_pose = Pose()
        object_one_pose.position.x = 0.0
        object_one_pose.position.y = 0.0
        object_one_pose.position.z = 0.0
        object_one_pose.orientation.x = 0.0
        object_one_pose.orientation.y = 0.0
        object_one_pose.orientation.z = 0.0
        object_one_pose.orientation.w = 1.0

        object_two_pose = Pose()
        object_two_pose.position.x = 0.0
        object_two_pose.position.y = 0.0
        object_two_pose.orientation.x = 0.0
        object_two_pose.orientation.y = 0.9990483518732844
        object_two_pose.orientation.z = 0.0
        object_two_pose.orientation.w = -0.043616403098766035

        threshold_distance = 1.0

        result = proximity(object_one_pose, object_two_pose,
                           threshold_distance)
        self.assertEquals(result, 1)

    def test_within_threhsold(self):
        object_one_pose = Pose()
        object_one_pose.position.x = 0.0
        object_one_pose.position.y = 0.0
        object_one_pose.position.z = 0.0
        object_one_pose.orientation.x = 0.0
        object_one_pose.orientation.y = 0.0
        object_one_pose.orientation.z = 0.0
        object_one_pose.orientation.w = 1.0

        object_two_pose = Pose()
        object_two_pose.position.x = 2.0
        object_two_pose.position.y = 0.0
        object_two_pose.position.z = 0.0
        object_two_pose.orientation.x = 0.0
        object_two_pose.orientation.y = 0.9990483518732844
        object_two_pose.orientation.z = 0.0
        object_two_pose.orientation.w = -0.043616403098766035

        threshold_distance = 3.0

        result = proximity(object_one_pose, object_two_pose,
                           threshold_distance)
        self.assertEquals(result, 1)

    def test_outside_threshold(self):
        object_one_pose = Pose()
        object_one_pose.position.y = 0.0
        object_one_pose.position.z = 0.0
        object_one_pose.orientation.x = 0.0
        object_one_pose.orientation.y = 0.0
        object_one_pose.orientation.z = 0.0
        object_one_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        object_two_pose = Pose()
        object_two_pose.position.x = 2.0
        object_two_pose.position.y = 0.0
        object_two_pose.position.z = 0.0
        object_two_pose.orientation.x = 0.0
        object_two_pose.orientation.y = 0.9990483518732844
        object_two_pose.orientation.z = 0.0
        object_two_pose.orientation.w = -0.043616403098766035

        threshold_distance = 1.0

        result = proximity(object_one_pose, object_two_pose,
                           threshold_distance)
        self.assertEquals(result, 0)


class TestHeightClassifier(unittest.TestCase):

    def test_above_within_threshold(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.0
        object_pose.position.z = 3.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 1.0
        threshold_distance = 4.0

        result = height(object_pose, reference_height, threshold_distance)
        self.assertEquals(result, 0)

    def test_below_within_threshold(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.0
        object_pose.position.z = -3.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 0.0
        threshold_distance = 4.0

        result = height(object_pose, reference_height, threshold_distance)
        self.assertEquals(result, 0)

    def test_above_outside_threshold(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.0
        object_pose.position.z = 5.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 2.0
        threshold_distance = 3.0

        result = height(object_pose, reference_height, threshold_distance)
        self.assertEquals(result, 1)

    def test_below_outside_threshold(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.0
        object_pose.position.z = -5.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = -1.0
        threshold_distance = 2.0

        result = height(object_pose, reference_height, threshold_distance)
        self.assertEquals(result, 1)

    def test_above_within_threshold_x_axis(self):
        object_pose = Pose()
        object_pose.position.x = 2.0
        object_pose.position.y = 0.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 1.0
        threshold_distance = 4.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='x')
        self.assertEquals(result, 0)

    def test_below_within_threshold_x_axis(self):
        object_pose = Pose()
        object_pose.position.x = -2.0
        object_pose.position.y = 0.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = -1.0
        threshold_distance = 4.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='x')
        self.assertEquals(result, 0)

    def test_above_outside_threshold_x_axis(self):
        object_pose = Pose()
        object_pose.position.x = 5.0
        object_pose.position.y = 0.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 2.0
        threshold_distance = 3.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='x')
        self.assertEquals(result, 1)

    def test_below_outside_threshold_x_axis(self):
        object_pose = Pose()
        object_pose.position.x = -5.0
        object_pose.position.y = 0.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = -1.0
        threshold_distance = 2.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='x')
        self.assertEquals(result, 1)

    def test_above_within_threshold_y_axis(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 3.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 1.0
        threshold_distance = 4.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='y')
        self.assertEquals(result, 0)

    def test_below_within_threshold_y_axis(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = -3.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = -1.0
        threshold_distance = 4.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='y')
        self.assertEquals(result, 0)

    def test_above_outside_threshold_y_axis(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 5.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = 2.0
        threshold_distance = 3.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='y')
        self.assertEquals(result, 1)

    def test_below_outside_threshold_y_axis(self):
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 5.0
        object_pose.position.z = 0.0
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        reference_height = -1.0
        threshold_distance = 2.0

        result = height(object_pose, reference_height,
                        threshold_distance, axis='y')
        self.assertEquals(result, 1)


class TestOverUnderClassifier(unittest.TestCase):

    def test_above_within_threshold(self):
        above_pose = Pose()
        above_pose.position.x = 1.2
        above_pose.position.y = 1.2
        above_pose.position.z = 1.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = 1.0

        threshold_distance = 3.0

        result = over_under(above_pose, below_pose, threshold_distance)
        self.assertEquals(result, 1)

    def test_above_outside_threshold(self):
        above_pose = Pose()
        above_pose.position.x = 1.0
        above_pose.position.y = 3.0
        above_pose.position.z = 1.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = 1.0

        threshold_distance = 1.0

        result = over_under(above_pose, below_pose, threshold_distance)
        self.assertEquals(result, 0)

    def test_below(self):
        above_pose = Pose()
        above_pose.position.x = 0.0
        above_pose.position.y = 0.0
        above_pose.position.z = 0.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = -1.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = 1.0

        threshold_distance = 1.0

        result = over_under(above_pose, below_pose, threshold_distance)
        self.assertEquals(result, 0)

    def test_above_within_threshold_x_axis(self):
        above_pose = Pose()
        above_pose.position.x = 1.0
        above_pose.position.y = 1.2
        above_pose.position.z = 1.2
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = 1.0

        threshold_distance = 3.5

        result = over_under(above_pose, below_pose,
                            threshold_distance, axis='x')
        self.assertEquals(result, 1)

    def test_above_outside_threshold_x_axis(self):
        above_pose = Pose()
        above_pose.position.x = 1.0
        above_pose.position.y = 3.0
        above_pose.position.z = 2.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = 1.0

        threshold_distance = 2.0

        result = over_under(above_pose, below_pose,
                            threshold_distance, axis='x')
        self.assertEquals(result, 0)

    def test_below_x_axis(self):
        above_pose = Pose()
        above_pose.position.x = -1.0
        above_pose.position.y = 0.0
        above_pose.position.z = 0.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = -0.043616403098766035

        threshold_distance = 1.0

        result = over_under(above_pose, below_pose,
                            threshold_distance, axis='x')
        self.assertEquals(result, 0)

    def test_above_within_threshold_y_axis(self):
        above_pose = Pose()
        above_pose.position.x = 0.0
        above_pose.position.y = 1.0
        above_pose.position.z = 1.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = 1.0

        threshold_distance = 2.5

        result = over_under(above_pose, below_pose,
                            threshold_distance, axis='y')
        self.assertEquals(result, 1)

    def test_above_outside_threshold_y_axis(self):
        above_pose = Pose()
        above_pose.position.x = 0.0
        above_pose.position.y = 0.0
        above_pose.position.z = 1.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 3.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = -0.043616403098766035

        threshold_distance = 1.0

        result = over_under(above_pose, below_pose,
                            threshold_distance, axis='y')
        self.assertEquals(result, 0)

    def test_below_y_axis(self):
        above_pose = Pose()
        above_pose.position.x = 0.0
        above_pose.position.y = 0.0
        above_pose.position.z = 0.0
        above_pose.orientation.x = 0.0
        above_pose.orientation.y = 0.0
        above_pose.orientation.z = 0.0
        above_pose.orientation.w = 1.0

        # R: 0, P: 175, Y: 0
        below_pose = Pose()
        below_pose.position.x = 0.0
        below_pose.position.y = 0.0
        below_pose.position.z = 0.0
        below_pose.orientation.x = 0.0
        below_pose.orientation.y = 0.0
        below_pose.orientation.z = 0.0
        below_pose.orientation.w = -0.043616403098766035

        threshold_distance = 25

        result = over_under(above_pose, below_pose,
                            threshold_distance, axis='y')
        self.assertEquals(result, 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('predicate_classication',
                    'test_upright_classifier', __name__, sys.argv)
