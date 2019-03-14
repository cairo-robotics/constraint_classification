from predicate_classification.utils import pt_in_polygon

def perimeter(pose, inner_polygon, outer_polygon, axis="z"):
	if axis == "z":
		p1 = pose.position.x
		p2 = pose.position.y
	elif axis == "y":
		p1 = pose.position.x
		p2 = pose.position.z
	elif axis == "x":
		p1 = pose.position.y
		p2 = pose.position.z
		
	if (pt_in_polygon(p1,p2,outer_polygon) is True) and (pt_in_polygon(p1,p2,inner_polygon) is False):
		return 1
	else:
		return 0