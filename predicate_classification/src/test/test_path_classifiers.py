#!/usr/bin/env python

import sys
import unittest
from geometry_msgs.msg import Pose
from predicate_classification.path_classifiers import perimeter


class TestPerimeterConstraint(unittest.TestCase):

	def test_square_perimeter(self):

		"""
    	Defines close polygon with hole in the center with inner and outer boundaries as squares.
    	Calls the "perimeter" function with 3 different coordinates, one outside the outer bounary, 
    	one between the inner and outer boundaries, and one inside the inner boundary,
    	and checks that the "perimeter" function returns correct values.

   		Parameters
    	----------
    	None.

    	Returns
    	-------
    	: None.
    	"""

		outer_polygon = [[-8,-8],[-8,8],[8,8],[8,-8]]
		inner_polygon = [[-4,-4],[-4,4],[4,4],[4,-4]]

		pose1 = Pose()
		pose1.position.x = 0.0
		pose1.position.y = 0.0
		pose1.position.z = 0.0
		pose1.orientation.x = 0.0
		pose1.orientation.y = 0.0
		pose1.orientation.z = 0.0
		pose1.orientation.w = 0.0

		pose2 = Pose()
		pose2.position.x = 0.0
		pose2.position.y = 6.0
		pose2.position.z = 0.0
		pose2.orientation.x = 0.0
		pose2.orientation.y = 0.0
		pose2.orientation.z = 0.0
		pose2.orientation.w = 0.0

		pose3 = Pose()
		pose3.position.x = 0.0
		pose3.position.y = 10.0
		pose3.position.z = 0.0
		pose3.orientation.x = 0.0
		pose3.orientation.y = 0.0
		pose3.orientation.z = 0.0
		pose3.orientation.w = 0.0

		result1 = perimeter(pose1, inner_polygon, outer_polygon)
		self.assertEquals(result1, 0)

		result2 = perimeter(pose2, inner_polygon, outer_polygon)
		self.assertEquals(result2, 1)

		result3 = perimeter(pose3, inner_polygon, outer_polygon)
		self.assertEquals(result3, 0)

	def test_rectangle_perimeter(self):

		"""
    	Defines close polygon with hole in the center with inner and outer boundaries as rectangles.
    	Calls the "perimeter" function with 3 different coordinates, one outside the outer bounary, 
    	one between the inner and outer boundaries, and one inside the inner boundary,
    	and checks that the "perimeter" function returns correct values.

   		Parameters
    	----------
    	None.

    	Returns
    	-------
    	: None.
    	"""

		outer_polygon = [[-8,-8],[-8,8],[16,8],[16,-8]]
		inner_polygon = [[-4,-4],[-4,4],[12,4],[12,-4]]

		pose1 = Pose()
		pose1.position.x = 0.0
		pose1.position.y = 0.0
		pose1.position.z = 0.0
		pose1.orientation.x = 0.0
		pose1.orientation.y = 0.0
		pose1.orientation.z = 0.0
		pose1.orientation.w = 0.0

		pose2 = Pose()
		pose2.position.x = 0.0
		pose2.position.y = 6.0
		pose2.position.z = 0.0
		pose2.orientation.x = 0.0
		pose2.orientation.y = 0.0
		pose2.orientation.z = 0.0
		pose2.orientation.w = 0.0

		pose3 = Pose()
		pose3.position.x = 0.0
		pose3.position.y = 10.0
		pose3.position.z = 0.0
		pose3.orientation.x = 0.0
		pose3.orientation.y = 0.0
		pose3.orientation.z = 0.0
		pose3.orientation.w = 0.0
		
		result1 = perimeter(pose1, inner_polygon, outer_polygon)
		self.assertEquals(result1, 0)

		result2 = perimeter(pose2, inner_polygon, outer_polygon)
		self.assertEquals(result2, 1)

		result3 = perimeter(pose3, inner_polygon, outer_polygon)
		self.assertEquals(result3, 0)


	def test_circle_perimeter(self):

		"""
    	Defines close polygon with hole in the center with inner and outer boundaries as circles.
    	Calls the "perimeter" function with 3 different coordinates, one outside the outer bounary, 
    	one between the inner and outer boundaries, and one inside the inner boundary,
    	and checks that the "perimeter" function returns correct values.

   		Parameters
    	----------
    	None.

    	Returns
    	-------
    	: None.
    	"""

		outer_polygon = [[0,-4],[-4,0],[0,4],[4,0]]
		inner_polygon = [[0,-2],[-2,0],[0,2],[2,0]]

		pose1 = Pose()
		pose1.position.x = 0.0
		pose1.position.y = 0.0
		pose1.position.z =	 0.0
		pose1.orientation.x = 0.0
		pose1.orientation.y = 0.0
		pose1.orientation.z = 0.0
		pose1.orientation.w = 0.0

		pose2 = Pose()
		pose2.position.x = 0.0
		pose2.position.y = -3.0
		pose2.position.z = 0.0
		pose2.orientation.x = 0.0
		pose2.orientation.y = 0.0
		pose2.orientation.z = 0.0
		pose2.orientation.w = 0.0

		pose3 = Pose()
		pose3.position.x = 0.0
		pose3.position.y = -5.0
		pose3.position.z = 0.0
		pose3.orientation.x = 0.0
		pose3.orientation.y = 0.0
		pose3.orientation.z = 0.0
		pose3.orientation.w = 0.0

		result1 = perimeter(pose1, inner_polygon, outer_polygon)
		self.assertEquals(result1, 0)

		result2 = perimeter(pose2, inner_polygon, outer_polygon)
		self.assertEquals(result2, 1)

		result3 = perimeter(pose3, inner_polygon, outer_polygon)
		self.assertEquals(result3, 0)

	def test_triangle_perimeter(self):

		"""
    	Defines close polygon with hole in the center with inner and outer boundaries as triangles.
    	Calls the "perimeter" function with 3 different coordinates, one outside the outer bounary, 
    	one between the inner and outer boundaries, and one inside the inner boundary,
    	and checks that the "perimeter" function returns correct values.

   		Parameters
    	----------
    	None.

    	Returns
    	-------
    	: None.
    	"""

		outer_polygon = [[-6,-2],[0,5],[6,-2]]
		inner_polygon = [[-3,-1],[0,3],[3,-1]]

		pose1 = Pose()
		pose1.position.x = 0.0
		pose1.position.y = 0.0
		pose1.position.z = 0.0
		pose1.orientation.x = 0.0
		pose1.orientation.y = 0.0
		pose1.orientation.z = 0.0
		pose1.orientation.w = 0.0

		pose2 = Pose()
		pose2.position.x = 0.0
		pose2.position.y = 4.0
		pose2.position.z = 0.0
		pose2.orientation.x = 0.0
		pose2.orientation.y = 0.0
		pose2.orientation.z = 0.0
		pose2.orientation.w = 0.0

		pose3 = Pose()
		pose3.position.x = 0.0
		pose3.position.y = 6.0
		pose3.position.z = 0.0
		pose3.orientation.x = 0.0
		pose3.orientation.y = 0.0
		pose3.orientation.z = 0.0
		pose3.orientation.w = 0.

		result1 = perimeter(pose1, inner_polygon, outer_polygon)
		self.assertEquals(result1, 0)

		result2 = perimeter(pose2, inner_polygon, outer_polygon)
		self.assertEquals(result2, 1)

		result3 = perimeter(pose3, inner_polygon, outer_polygon)
		self.assertEquals(result3, 0)

	def test_star_perimeter(self):

		"""
    	Defines close polygon with hole in the center with inner and outer boundaries as 5-point stars.
    	Calls the "perimeter" function with 3 different coordinates, one outside the outer bounary, 
    	one between the inner and outer boundaries, and one inside the inner boundary,
    	and checks that the "perimeter" function returns correct values.

   		Parameters
    	----------
    	None.

    	Returns
    	-------
    	: None.
    	"""

		outer_polygon = [[-10,-10],[-5,0],[-14,8],[-4,8],[0,14],[4,8],[14,8],[6,0],[10,-10],[0,-6]]
		inner_polygon = [[-2,0],[-1,1],[-2,2],[-1,2],[0,3],[1,2],[2,2],[1,1],[2,0],[0,1]]

		pose1 = Pose()
		pose1.position.x = 0.0
		pose1.position.y = 2.0
		pose1.position.z = 0.0
		pose1.orientation.x = 0.0
		pose1.orientation.y = 0.0
		pose1.orientation.z = 0.0
		pose1.orientation.w = 0.0

		pose2 = Pose()
		pose2.position.x = 0.0
		pose2.position.y = 6.0
		pose2.position.z = 0.0
		pose2.orientation.x = 0.0
		pose2.orientation.y = 0.0
		pose2.orientation.z = 0.0
		pose2.orientation.w = 0.0

		pose3 = Pose()
		pose3.position.x = 0.0
		pose3.position.y = 15.0
		pose3.position.z = 0.0
		pose3.orientation.x = 0.0
		pose3.orientation.y = 0.0
		pose3.orientation.z = 0.0
		pose3.orientation.w = 0.0

		result1 = perimeter(pose1, inner_polygon, outer_polygon)
		self.assertEquals(result1, 0)

		result2 = perimeter(pose2, inner_polygon, outer_polygon)
		self.assertEquals(result2, 1)

		result3 = perimeter(pose3, inner_polygon, outer_polygon)
		self.assertEquals(result3, 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('predicate_classification',
                    'test_perimeter_constraint', __name__, sys.argv)
