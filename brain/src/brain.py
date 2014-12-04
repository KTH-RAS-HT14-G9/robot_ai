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

RobotDirections = IntEnum('RobotDirections','FORWARD RIGHT BACKWARDS LEFT')
MapDirections = IntEnum('MapDirections','NORTH EAST SOUTH WEST')

ROBOT_DIAMETER = 0.25
RECOGNITION_TIME = 2.0
WAITING_TIME = 0.005
DIRECTION_BLOCKED = -2
SIDE_BLOCKED_THRESHOLD = 0.35
FRONT_BLOCKED_THRESHOLD = 0.25

detected_object = Object()
odometry = Odometry()
distance = Distance()
current_node = Node()

current_direction = MapDirections.EAST
reset_mc_pub = None
recognize_object_pub = None
turn_pub = None
follow_wall_pub = None
go_forward_pub = None
place_node_service = None
next_noi_service = None
fit_blob_service = None

turn_done = False
object_recognized = False
object_detected = False
following_wall = False
going_forward = False
stop_done = False
fetch_objects = False
walls_have_changed = True
node_detected = False

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore','obstacle_detected', 'object_detected', 'follow_graph', 'finished'])

    def execute(self, userdata):
        check_for_interrupt()
        node_seen = node_detected
        reset_node_detected()
        follow_wall(True)
        go_forward(True)
        if object_detected and not recognized_before():
            rospy.loginfo("EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        elif node_seen:
            rospy.loginfo("EXPLORE ==> FOLLOW_GRAPH")
            return 'follow_graph'
        elif obstacle_ahead():
            rospy.loginfo("EXPLORE ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'        
        elif is_at_intersection():
            rospy.loginfo("Is at intersection, placing node")
            place_node(False)
        return 'explore'    

class ObstacleDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore','obstacle_detected'])

    def execute(self, userdata):
        #if not stop_done:
            go_forward(False)
            follow_wall(False)
          #  return 'obstacle_detected'
        #else:
            place_node(False)
            if can_turn_left():
                turn_left()
            elif can_turn_right():
                turn_right()
            else:
                turn_back()
            rospy.loginfo("OBSTACLE_DETECTED ==> EXPLORE")
            reset_node_detected()
            return 'explore'

class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore'])

    def execute(self, userdata):

        go_forward(False)
        follow_wall(False)

        object_angle=atan2(detected_object.y,detected_object.x)
        object_angle=180*(object_angle/pi)

        has_turned = False
        if (fabs(object_angle)>10):
           turn(object_angle)
           has_turned = True
        
        object_recognized = recognize_object()
        
        if has_turned:
            turn(-object_angle)
        
        if object_recognized:
            place_node(True)

        rospy.loginfo("OBJECT_DETECTED ==> EXPLORE")
        reset_node_detected()
        return 'explore'

class FollowGraph(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore', 'follow_graph'])

    def execute(self, userdata):
        
        next_node = get_next_noi()
        if(next_node.id_this == current_node.id_this):
            rospy.loginfo("Destination reached")
            rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
            return 'explore'
        angle = get_angle_to(get_direction_to(next_node))
        if angle != 0:
            go_forward(False)
            follow_wall(False)
            turn(angle)

        follow_wall(True)
        go_forward(True)

        node_detected = False
        while not node_detected:
            if obstacle_ahead():
                rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
                return 'explore'
            check_for_interrupt()
            rospy.sleep(WAITING_TIME)
        return 'follow_graph'

def check_for_interrupt():
    if rospy.is_shutdown():
        return sys.exit(0);

def obstacle_behind():
    free = fit_blob_service.call(-ROBOT_DIAMETER+0.03, 0.0, 0.08, 0.05)
    return not free

def get_direction_to(node):
    if node.id_north == current_node.id_this:
        return MapDirections.SOUTH
    if node.id_east == current_node.id_this:
        return MapDirections.WEST
    if node.id_south == current_node.id_this:
        return MapDirections.NORTH
    if node.id_west == current_node.id_this:
        return MapDirections.EAST

def get_angle_to(map_dir):
    angle = 90.0 * ((current_direction - map_dir + 4) % 4)
    if angle == 270.0:
        return -90.0
    return angle

def recognized_before():
    global object_detected
    object_detected = False
    return current_node.object_here

def get_next_noi():
    mode = 0
    if fetch_objects:
        mode = 1
    return next_noi_service.call(NextNodeOfInterestRequest(current_node.id_this, mode)).target_node

def place_node(object_here):
    global current_node, walls_have_changed
    walls_have_changed = False
    response = place_node_service.call(PlaceNodeRequest(current_node.id_this, current_direction, north_blocked(), east_blocked(), south_blocked(), west_blocked(), object_here, detected_object.type, current_direction, detected_object.x, detected_object.y))
    current_node = response.generated_node

def north_blocked():
    return map_dir_blocked(MapDirections.NORTH)

def west_blocked():
    return map_dir_blocked(MapDirections.WEST)

def south_blocked():
    return map_dir_blocked(MapDirections.SOUTH)

def east_blocked():
    return map_dir_blocked(MapDirections.EAST)

def reset_node_detected():
    node_detected = False

def can_turn_left():
    return True if distance.fl_side > SIDE_BLOCKED_THRESHOLD and distance.bl_side > SIDE_BLOCKED_THRESHOLD else False

def can_turn_right():
    return True if distance.fr_side > SIDE_BLOCKED_THRESHOLD and distance.br_side > SIDE_BLOCKED_THRESHOLD else False

def obstacle_ahead():
    return True if distance.l_front < FRONT_BLOCKED_THRESHOLD or distance.r_front < FRONT_BLOCKED_THRESHOLD else False

def turn_left():
    rospy.loginfo("turning left")
    turn(90.0)
    
def turn_right():
    rospy.loginfo("turning right")
    turn(-90.0)

def turn_back():
    rospy.loginfo("turning back")
    turn(180.0)

def turn(angle):
    global turn_done
    reset_motor_controller()
    turn_done = False
    turn_pub.publish(angle)
    wait_for_flag(turn_done)
    rospy.loginfo("turn done")
    update_direction(angle)
    rospy.sleep(0.5) # for aligning

def update_direction(turn_angle):
    global current_direction

    increment = 0
    if turn_angle == 90.0:
        increment = 3
    if turn_angle == -90.0:
        increment = 1
    if turn_angle == 180.0:
        increment = 2

    current_direction = (current_direction + increment) % 4

def wait_for_flag(flag):
    while not flag:
        check_for_interrupt()
        rospy.sleep(WAITING_TIME)

def follow_wall(should_follow):
    global following_wall
    if (should_follow and not following_wall) or (not should_follow and following_wall):
        following_wall = not following_wall
        follow_wall_pub.publish(should_follow)
        rospy.loginfo("Following Wall: %s", str(should_follow))

def go_forward(should_go):
    global going_forward, stop_done

    if should_go != going_forward:
        reset_motor_controller()
        going_forward = should_go
        stop_done = False
        go_forward_pub.publish(should_go)
        rospy.loginfo("Going forward: %s", str(should_go))
        if not should_go:
            wait_for_flag(stop_done)
            rospy.loginfo("Stopping Done")

def recognize_object():
    global object_detected, object_recognized

    object_recognized = False
    rospy.loginfo("Start recognizing object")
    recognize_object_pub.publish(True)

    rospy.sleep(RECOGNITION_TIME)
    if object_recognized:
        rospy.loginfo("Object recognized")
    else:
        rospy.loginfo("Recognition Failed")

    object_recognized = False
    object_detected = False
    
def reset_motor_controller():
    reset_mc_pub.publish(True)
    rospy.loginfo("Resetting MC pid")

def update_walls_changed():
    global walls_have_changed
    if not ((current_node.id_north==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.NORTH) and 
            (current_node.id_east==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.EAST) and
            (current_node.id_south==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.SOUTH) and 
            (current_node.id_west==DIRECTION_BLOCKED) == map_dir_blocked(MapDirections.WEST)):
        walls_have_changed = True

def walls_changed():
    return walls_have_changed

def is_at_intersection():
    if walls_changed() and not obstacle_ahead() and (can_turn_right() or can_turn_left()):
            rospy.loginfo("Placing node at intersection.")
            rospy.loginfo("Prev node: N: %s, E: %s, S: %s, W: %s", str(current_node.id_north == DIRECTION_BLOCKED), str(current_node.id_east == DIRECTION_BLOCKED), str(current_node.id_south == DIRECTION_BLOCKED), str(current_node.id_west == DIRECTION_BLOCKED))
            rospy.loginfo("Currently: N: %s, E: %s, S: %s, W: %s", str(robot_dir_blocked(map_to_robot_dir(MapDirections.NORTH))), str(robot_dir_blocked(map_to_robot_dir(MapDirections.EAST))), str(robot_dir_blocked(map_to_robot_dir(MapDirections.SOUTH))), str(robot_dir_blocked(map_to_robot_dir(MapDirections.WEST))))
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
    if current_direction == MapDirections.NORTH:
        return robot_dir
    if current_direction == MapDirections.WEST:
        return (robot_dir + 3) % 4
    if current_direction == MapDirections.SOUTH:
        return (robot_dir + 2) % 4
    return (robot_dir + 1) % 4 

def map_to_robot_dir(map_dir):
    if current_direction == MapDirections.NORTH:
        return map_dir
    if current_direction == MapDirections.WEST:
        return (map_dir + 1) % 4
    if current_direction == MapDirections.SOUTH:
        return (map_dir + 2) % 4
    return (map_dir + 3) % 4

def turn_done_callback(data):
    global turn_done
    turn_done = True
    rospy.loginfo("turn done callback: %s", str(data))

def stopping_done_callback(data):
    global stop_done
    stop_done = True
    rospy.loginfo("Stopping done callback: %s", str(data))

def object_recognized_callback(data):
    global object_recognized
    object_recognized = True
    rospy.loginfo("Object Recognized: %s", str(data))

def ir_callback(data):
    global distance
    distance = data
    update_walls_changed()

def object_detected_callback(data):
    global detected_object, object_detected
    detected_object = data
    object_detected = True
    rospy.loginfo("Object detected")

def on_node_callback(node):
    global current_node
    
    if(current_node.id_this == node.id_this):
        current_node = node
        node_detected = True

def odometry_callback(data):
    global odometry
    odometry = data

def main(argv):
    global turn_pub, follow_wall_pub, go_forward_pub, recognize_object_pub, reset_mc_pub, fetch_objects, place_node_service, next_noi_service, current_node
    rospy.init_node('brain')

    if len(argv) > 1 and argv[1] == 'fetch':
        fetch_objects = True
    
    sm = smach.StateMachine(outcomes=['finished'])
    rospy.Subscriber("/perception/ir/distance", Distance, ir_callback)
    rospy.Subscriber("/controller/turn/done", Bool, turn_done_callback)
    rospy.Subscriber("/vision/recognition/done", String, object_recognized_callback) 
    rospy.Subscriber("/vision/obstacle/object", Object, object_detected)
    rospy.Subscriber("/controller/forward/stopped", Bool, stopping_done_callback)
    rospy.Subscriber("/navigation/graph/on_node", Node, on_node_callback)
    rospy.Subscriber("/pose/odometry/", Odometry, odometry_callback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=10)
    follow_wall_pub = rospy.Publisher("/controller/wall_follow/active", Bool, queue_size=10)
    go_forward_pub = rospy.Publisher("/controller/forward/active", Bool, queue_size=10)
    recognize_object_pub = rospy.Publisher("/vision/recognition/active", Bool, queue_size=10)
    reset_mc_pub = rospy.Publisher("controller/motor/reset", Bool, queue_size=1)

    with sm:
        smach.StateMachine.add('EXPLORE', Explore(), transitions={'explore':'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED', 'follow_graph' : 'FOLLOW_GRAPH', 
            'object_detected' : 'OBJECT_DETECTED', 'finished':'finished'})
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
