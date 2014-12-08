#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import sys
import time
import math
from std_msgs.msg import *
from navigation_msgs.msg import *
from navigation_msgs.srv import *
from ir_converter.msg import Distance
from vision_msgs.msg import Object
from nav_msgs.msg import Odometry
from enum import IntEnum

class MapDirections(IntEnum):
    NORTH=0
    EAST=1
    SOUTH=2
    WEST=3
    OBJECT=4

class RobotDirections(IntEnum):
    FORWARD=0
    RIGHT=1
    BACKWARDS=2
    LEFT=3

ROBOT_DIAMETER = 0.25
RECOGNITION_TIME = 2.0
WAITING_TIME = 0.005
DIRECTION_BLOCKED = -2
SIDE_BLOCKED_THRESHOLD = 0.35
FRONT_BLOCKED_THRESHOLD = 0.23

detected_object = Object()
recognition_clock = 0.0
odometry = Odometry()
distance = Distance()
current_node = Node()

compass_direction = MapDirections.EAST

go_to_node_pub = None
recognize_object_pub = None
turn_pub = None
follow_wall_pub = None
go_forward_pub = None
place_node_service = None
next_noi_service = None
fit_blob_service = None

turn_done = [False]
object_recognized = False
object_detected = False
following_wall = False
going_forward = False
stop_done = [False]
walls_have_changed = True
node_detected = False
go_to_node_done = [False]

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['m3_explore','explore','obstacle_detected', 'object_detected', 'follow_graph'])

    def execute(self, userdata):
        return 'm3_explore'
        check_for_interrupt()
        node_seen = node_detected
        reset_node_detected()
        follow_wall(True)
        go_forward(True)
        update_walls_changed()
        if object_detected:
            rospy.loginfo("EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        elif node_seen:
            go_forward(False)
            follow_wall(False)
            rospy.loginfo("EXPLORE ==> FOLLOW_GRAPH")
            return 'follow_graph'
        elif obstacle_ahead():
            rospy.loginfo("EXPLORE ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'        
        elif is_at_intersection():
            place_node(False)
        return 'explore'    

class ObstacleDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore','obstacle_detected'])

    def execute(self, userdata):

            go_forward(False)
            follow_wall(False)
            if can_turn_left():
                turn_left()
            elif can_turn_right():
                turn_right()
            else:
                turn_back()
            rospy.loginfo("OBSTACLE_DETECTED ==> EXPLORE")
            place_node(False)

            return 'explore'

class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore'])

    def execute(self, userdata):
        if detected_object.type == -1:

            rospy.loginfo("Unknown object seen, trying to recognize")
            go_forward(False)
            follow_wall(False)

            object_angle=math.atan2(detected_object.y,detected_object.x)
            object_angle=180*(object_angle/math.pi)

            has_turned = False
            if (math.fabs(object_angle)>10):
               turn(object_angle)
               has_turned = True
            
            rospy.sleep(RECOGNITION_TIME)

            if detected_object.type == -1 and (rospy.get_time() - recognition_clock) > 5.0:
                rospy.loginfo("Reognition failed")
            else: rospy.loginfo("Reognition successful")
            
            if has_turned:
                turn(-object_angle)

        if detected_object.type != -1:
            place_node(True)

        rospy.loginfo("OBJECT_DETECTED ==> EXPLORE")
        reset_node_detected()
        return 'explore'

class FollowGraph(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore', 'follow_graph'])

    def execute(self, userdata):
        reset_node_detected()
        next_node = get_next_noi(0)
        if(next_node.id_this == current_node.id_this):
            go_forward(False)
            follow_wall(False)
            rospy.loginfo("Destination reached")
            rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
            return 'explore'
        
        go_to_node(next_node)
        wait_for_flag(go_to_node_done)

        return 'explore'

def check_for_interrupt():
    if rospy.is_shutdown():
        return sys.exit(0)

def get_direction_to(node):
    if node.edges[MapDirections.NORTH] == current_node.id_this:
        return MapDirections.SOUTH
    if node.edges[MapDirections.EAST] == current_node.id_this:
        return MapDirections.WEST
    if node.edges[MapDirections.SOUTH] == current_node.id_this:
        return MapDirections.NORTH
    if node.edges[MapDirections.WEST] == current_node.id_this:
        return MapDirections.EAST
    rospy.logerr("Next node is not neighbour to this node. current: %s, goal: %s", str(current_node), str(node))

#def get_angle_to(map_dir):
#    angle = 90.0 * ((compass_direction - map_dir + 4) % 4)
#    if angle == 270.0:
#        return -90.0
#    return angle

def get_next_noi(mode):
    return next_noi_service.call(NextNodeOfInterestRequest(current_node.id_this, mode)).target_node

def place_node(object_here):
    global current_node, walls_have_changed
    walls_have_changed = False
    response = place_node_service.call(PlaceNodeRequest(current_node.id_this, compass_direction, north_blocked(), east_blocked(), south_blocked(), west_blocked(), object_here, detected_object.type, detected_object.x, detected_object.y))
    if not object_here:
        current_node = response.generated_node

def north_blocked():
    return map_dir_blocked(MapDirections.NORTH)

def west_blocked():
    return map_dir_blocked(MapDirections.WEST)

def south_blocked():
    return map_dir_blocked(MapDirections.SOUTH)

def east_blocked():
    return map_dir_blocked(MapDirections.EAST)

def can_turn_left():
    return True if distance.fl_side > SIDE_BLOCKED_THRESHOLD and distance.bl_side > SIDE_BLOCKED_THRESHOLD else False

def can_turn_right():
    return True if distance.fr_side > SIDE_BLOCKED_THRESHOLD and distance.br_side > SIDE_BLOCKED_THRESHOLD else False

def obstacle_ahead():
    return True if distance.l_front < FRONT_BLOCKED_THRESHOLD or distance.r_front < FRONT_BLOCKED_THRESHOLD else False

def obstacle_behind():
    response = fit_blob_service.call(FitBlobRequest(-ROBOT_DIAMETER+0.03, 0.0, 0.08, 0.05))
    return not response.fits

def reset_node_detected():
    global node_detected
    node_detected = False

def turn_left():
    rospy.loginfo("Turning left.")
    turn(90.0)
    
def turn_right():
    rospy.loginfo("Turning right.")
    turn(-90.0)

def turn_back():
    rospy.loginfo("Turning back.")
    turn(180.0)

def turn(angle):
    global turn_done
    if math.fabs(angle) < 1.0:
        return
    turn_done[0] = False
    turn_pub.publish(angle)
    wait_for_flag(turn_done)
    rospy.loginfo("turn done")
    rospy.sleep(0.5) # for aligning

def wait_for_flag(flag):
    while not flag[0]:
        check_for_interrupt()
        rospy.sleep(WAITING_TIME)

def follow_wall(should_follow):
    global following_wall
    if should_follow != following_wall:
        following_wall = should_follow
        follow_wall_pub.publish(should_follow)

def go_forward(should_go):
    global going_forward, stop_done

    if should_go != going_forward:
        going_forward = should_go
        stop_done[0] = False
        go_forward_pub.publish(should_go)
        if not should_go:
            wait_for_flag(stop_done)

def update_walls_changed():
    global walls_have_changed
    if not ((current_node.edges[MapDirections.NORTH]==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.NORTH) and 
            (current_node.edges[MapDirections.EAST] ==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.EAST) and
            (current_node.edges[MapDirections.SOUTH]==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.SOUTH) and 
            (current_node.edges[MapDirections.WEST] ==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.WEST)):
        walls_have_changed = True

def walls_changed():
    return walls_have_changed

def go_to_node(node):
    global go_to_node_done
    go_to_node_done[0] = False
    rospy.loginfo("Going to node: %d.", node.id_this)
    go_to_node_pub.publish(node)

def is_at_intersection():
    if walls_changed() and not obstacle_ahead() and (can_turn_right() or can_turn_left()):
            return True
    return False

def map_dir_blocked(map_dir):
    return robot_dir_blocked(map_to_robot_dir(map_dir))

def robot_dir_blocked(robot_dir):
    if robot_dir == RobotDirections.LEFT:
        return not can_turn_left()
    if robot_dir == RobotDirections.RIGHT:
        return not can_turn_right()
    if robot_dir == RobotDirections.FORWARD:
        return obstacle_ahead()
    if robot_dir == RobotDirections.BACKWARDS:
        return obstacle_behind()

def robot_to_map_dir(robot_dir):
    if compass_direction == MapDirections.NORTH:
        return robot_dir
    if compass_direction == MapDirections.EAST:
        return (robot_dir + 1) % 4 
    if compass_direction == MapDirections.SOUTH:
        return (robot_dir + 2) % 4
    if compass_direction == MapDirections.WEST:
        return (robot_dir + 3) % 4

def map_to_robot_dir(map_dir):
    if compass_direction == MapDirections.NORTH:
        return map_dir
    if compass_direction == MapDirections.WEST:
        return (map_dir + 1) % 4
    if compass_direction == MapDirections.SOUTH:
        return (map_dir + 2) % 4
    if compass_direction == MapDirections.EAST:
        return (map_dir + 3) % 4

def turn_done_callback(data):
    global turn_done
    turn_done[0] = True
    rospy.loginfo("turn done callback received.")

def stopping_done_callback(data):
    global stop_done
    stop_done[0] = True
    rospy.loginfo("Stopping done callback received.")

def ir_callback(data):
    global distance
    distance = data

def object_detected_callback(new_object):
    global detected_object, object_detected, recognition_clock
    if distance_between(new_object, detected_object) > 0.2 or (new_object.type != -1 and new_object.type != object_detected.type):
        rospy.loginfo("New Object detected X=%f, Y=%f, TYPE=%d.", new_object.x, new_object.y, new_object.type)
        detected_object = new_object
        object_detected = True

        if new_object.type != -1:
            recognition_clock = rospy.get_time()
    else: 
        rospy.loginfo("Familiar object detected X=%f, Y=%f, TYPE=%d.", new_object.x, new_object.y, new_object.type)
    

def distance_between(object1, object2):
    return math.sqrt(math.pow(object1.x-object2.x,2)+math.pow(object1.y-object2.y,2))

def on_node_callback(node):
    global current_node, node_detected
    if(current_node.id_this != node.id_this):
        rospy.loginfo("On node callback received. Previous node: %d, New node: %d.", current_node.id_this, node.id_this)
        current_node = node
        node_detected = True

def go_to_node_done_callback(success):   
    global go_to_node_done
    go_to_node_done[0] = success
    rospy.loginfo("Go to node callback received. Success: %s.", str(success))

def odometry_callback(data):
    global odometry
    odometry = data

def compass_callback(data):
    global compass_direction
    compass_direction = data

def main(argv):
    global turn_pub, follow_wall_pub, go_forward_pub, recognize_object_pub, place_node_service, next_noi_service, current_node, fit_blob_service, go_to_node_pub
    rospy.init_node('brain')

    sm = smach.StateMachine(outcomes=['finished'])
    rospy.Subscriber("/perception/ir/distance", Distance, ir_callback)
    rospy.Subscriber("/controller/turn/done", Bool, turn_done_callback)
    rospy.Subscriber("/vision/obstacle/object", Object, object_detected_callback)
    rospy.Subscriber("/controller/forward/stopped", Bool, stopping_done_callback)
    rospy.Subscriber("/navigation/graph/on_node", Node, on_node_callback)
    rospy.Subscriber("/pose/odometry", Odometry, odometry_callback)
    rospy.Subscriber("/pose/compass", Int8, compass_callback)
    rospy.Subscriber("/controller/goto/success", Bool, go_to_node_done_callback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=10)
    follow_wall_pub = rospy.Publisher("/controller/wall_follow/active", Bool, queue_size=10)
    go_forward_pub = rospy.Publisher("/controller/forward/active", Bool, queue_size=10)
    go_to_node_pub = rospy.Publisher("controller/goto/target_node", Node, queue_size=1)

    with sm:
        smach.StateMachine.add('EXPLORE', Explore(), transitions={'explore':'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED', 'follow_graph' : 'FOLLOW_GRAPH', 
            'object_detected' : 'OBJECT_DETECTED', 'm3_explore': 'M3_EXPLORE'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetected(), transitions={'explore': 'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED'})
        smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), transitions={'explore': 'EXPLORE'})
        smach.StateMachine.add('FOLLOW_GRAPH', FollowGraph(), transitions={'explore' : 'EXPLORE',
            'follow_graph' : 'FOLLOW_GRAPH'})

    rospy.wait_for_service('/navigation/graph/place_node')
    rospy.wait_for_service('/navigation/graph/next_node_of_interest')
    rospy.wait_for_service('/mapping/fitblob')
    place_node_service = rospy.ServiceProxy('/navigation/graph/place_node', navigation_msgs.srv.PlaceNode)
    next_noi_service = rospy.ServiceProxy('/navigation/graph/next_node_of_interest', navigation_msgs.srv.NextNodeOfInterest)
    fit_blob_service = rospy.ServiceProxy('/mapping/fitblob', navigation_msgs.srv.FitBlob)

    rospy.sleep(3.0)

    response = place_node_service.call(PlaceNodeRequest(-1, MapDirections.EAST, north_blocked(), east_blocked(), south_blocked(), True, False, -1, -1, -1, -1))
    current_node = response.generated_node

    outcome = sm.execute() 

#if __name__ == '__main__':
main(sys.argv)

