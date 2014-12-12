import direction_handler
import roslib
import rospy
from ir_converter.msg import Distance
from navigation_msgs.srv import *
from navigation_msgs.msg import *
from nav_msgs.msg import Odometry
from direction_handler import *

SIDE_BLOCKED_THRESHOLD = 0.30
FRONT_BLOCKED_THRESHOLD = 0.20
ROBOT_DIAMETER = 0.25
fit_blob_service = None
MAX_OCCLUSION = 0.1
MIN_UNSEEN = 0.3

class ObstacleHandler:

    compass_direction = Node.EAST
    distance = Distance()
    odometry = None

    rospy.wait_for_service('/mapping/has_unexplored_region')
    has_unexplored_region_service = rospy.ServiceProxy('/mapping/has_unexplored_region', navigation_msgs.srv.UnexploredRegion)
    rospy.wait_for_service('/mapping/fitblob')
    fit_blob_service = rospy.ServiceProxy('/mapping/fitblob', navigation_msgs.srv.FitBlob)

    @staticmethod
    def map_dir_blocked(map_dir):
        return ObstacleHandler.robot_dir_blocked(map_to_robot_dir(map_dir, ObstacleHandler.compass_direction))

    @staticmethod
    def robot_dir_blocked(robot_dir):
        if robot_dir == RobotDirections.LEFT:
            return not ObstacleHandler.can_turn_left()
        if robot_dir == RobotDirections.RIGHT:
            return not ObstacleHandler.can_turn_right()
        if robot_dir == RobotDirections.FORWARD:
            return ObstacleHandler.obstacle_ahead()
        if robot_dir == RobotDirections.BACKWARD:
            return ObstacleHandler.obstacle_behind()
    
    @staticmethod
    def north_blocked():
        #print >> sys.stderr, ObstacleHandler.odometry
        if ObstacleHandler.map_dir_blocked(Node.NORTH):
            return True
        
        response = ObstacleHandler.has_unexplored_region_service.call(UnexploredRegionRequest("map", ObstacleHandler.odometry.x, ObstacleHandler.odometry.y+ROBOT_DIAMETER-0.03, 0.08, MAX_OCCLUSION, MIN_UNSEEN))
        return not response.has_unexplored

    @staticmethod
    def east_blocked():
        if ObstacleHandler.map_dir_blocked(Node.EAST):
            return True

        response = ObstacleHandler.has_unexplored_region_service.call(UnexploredRegionRequest("map", ObstacleHandler.odometry.x+ROBOT_DIAMETER-0.03, ObstacleHandler.odometry.y, 0.08, MAX_OCCLUSION, MIN_UNSEEN))
        return not response.has_unexplored

    @staticmethod
    def south_blocked():
        if ObstacleHandler.map_dir_blocked(Node.SOUTH):
            return True

        response = ObstacleHandler.has_unexplored_region_service.call(UnexploredRegionRequest("map", ObstacleHandler.odometry.x, ObstacleHandler.odometry.y-(ROBOT_DIAMETER-0.03), 0.08, MAX_OCCLUSION, MIN_UNSEEN))
        return not response.has_unexplored

    @staticmethod
    def west_blocked():
        if ObstacleHandler.map_dir_blocked(Node.WEST):
            return True

        response = ObstacleHandler.has_unexplored_region_service.call(UnexploredRegionRequest("map", ObstacleHandler.odometry.x-(ROBOT_DIAMETER-0.03), ObstacleHandler.odometry.y, 0.08, MAX_OCCLUSION, MIN_UNSEEN))
        return not response.has_unexplored

    @staticmethod
    def can_turn_left():
        return True if ObstacleHandler.distance.fl_side > SIDE_BLOCKED_THRESHOLD and ObstacleHandler.distance.bl_side > SIDE_BLOCKED_THRESHOLD else False

    @staticmethod
    def can_turn_right():
        return True if ObstacleHandler.distance.fr_side > SIDE_BLOCKED_THRESHOLD and ObstacleHandler.distance.br_side > SIDE_BLOCKED_THRESHOLD else False

    @staticmethod
    def obstacle_ahead():
        if ObstacleHandler.distance.l_front < FRONT_BLOCKED_THRESHOLD or ObstacleHandler.distance.r_front < FRONT_BLOCKED_THRESHOLD:
            print("OBSTACLE", ObstacleHandler.distance)
        return True if ObstacleHandler.distance.l_front < FRONT_BLOCKED_THRESHOLD or ObstacleHandler.distance.r_front < FRONT_BLOCKED_THRESHOLD else False

    @staticmethod
    def obstacle_behind():
        response = ObstacleHandler.fit_blob_service.call(FitBlobRequest("robot", -ROBOT_DIAMETER+0.03, 0.0, 0.08, 0.05))
        return not response.fits

        
        
