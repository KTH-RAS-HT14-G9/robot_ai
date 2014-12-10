import direction_handler
import roslib
import rospy

SIDE_BLOCKED_THRESHOLD = 0.35
FRONT_BLOCKED_THRESHOLD = 0.23
ROBOT_DIAMETER = 0.25

rospy.wait_for_service('/mapping/fitblob')
fit_blob_service = rospy.ServiceProxy('/mapping/fitblob', navigation_msgs.srv.FitBlob)

def map_dir_blocked(map_dir):
	return robot_dir_blocked(map_to_robot_dir(map_dir))

def robot_dir_blocked(robot_dir):
    if robot_dir == RobotDirections.LEFT:
        return not can_turn_left()
    if robot_dir == RobotDirections.RIGHT:
        return not can_turn_right()
    if robot_dir == RobotDirections.FORWARD:
        return obstacle_ahead()
    if robot_dir == RobotDirections.BACKWARD:
        return obstacle_behind()

def north_blocked():
    return map_dir_blocked(Node.NORTH)

def west_blocked():
    return map_dir_blocked(Node.WEST)

def south_blocked():
    return map_dir_blocked(Node.SOUTH)

def east_blocked():
    return map_dir_blocked(Node.EAST)

def can_turn_left(distance):
    return True if distance.fl_side > SIDE_BLOCKED_THRESHOLD and distance.bl_side > SIDE_BLOCKED_THRESHOLD else False

def can_turn_right(distance):
    return True if distance.fr_side > SIDE_BLOCKED_THRESHOLD and distance.br_side > SIDE_BLOCKED_THRESHOLD else False

def obstacle_ahead(distance):
    return True if distance.l_front < FRONT_BLOCKED_THRESHOLD or distance.r_front < FRONT_BLOCKED_THRESHOLD else False

def obstacle_behind():
    response = fit_blob_service.call(FitBlobRequest(-ROBOT_DIAMETER+0.03, 0.0, 0.08, 0.05))
    return not response.fits

    	
