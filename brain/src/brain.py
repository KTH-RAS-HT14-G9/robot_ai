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

from direction_handler import *
from obstacle_handler import ObstacleHandler

OBJECT_DETECTION_MUTE_TIME = 5.0
RECOGNITION_TIME = 3.0
WAITING_TIME = 0.005

object_recognized_time = 0.0
recognition_done_time = 0.0

detected_object = Object()
odometry = Odometry()
distance = Distance()
current_node = Node()

compass_direction = Node.EAST

goto_node_pub = None
turn_pub = None
follow_wall_pub = None
go_forward_pub = None
mapping_active_pub = None
follow_path_pub = None
place_node_service = None
next_noi_service = None

turn_done = [False] # In array so it can be passed by reference
goto_done = [False]
stop_done = [False]
object_detected = False
following_wall = False
going_forward = False
walls_have_changed = True
node_detected = False

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=
                                    ['explore','obstacle_detected', 
                                    'object_detected', 'follow_graph'], 
                                    output_keys=['trait'])

    def execute(self, userdata):
        check_for_interrupt()
        follow_wall(True)
        go_forward(True)
        update_walls_changed()
        if object_detected:
            rospy.loginfo("EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        elif node_detected:
            userdata.trait = NextNodeOfInterestRequest.TRAIT_UNKNOWN_DIR
            rospy.loginfo("EXPLORE ==> FOLLOW_GRAPH")
            return 'follow_graph'
        elif obstacle_ahead():
            rospy.loginfo("EXPLORE ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'        
        elif is_at_intersection():
            rospy.loginfo("Intersection detected, placing node")
            place_node(False)
        return 'explore'    

class ObstacleDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore','obstacle_detected'])

    def execute(self, userdata):

            go_forward(False)
            follow_wall(False)
            place_node(False)

            if can_turn_left():
                turn_left()
            elif can_turn_right():
                turn_right()
            else:
                turn_back()
            rospy.loginfo("OBSTACLE_DETECTED ==> EXPLORE")

            return 'explore'

class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore'])

    def execute(self, userdata):
        global recognition_done_time, object_detected
        if detected_object.type == Object.TYPE_UNKNOWN:
            time = rospy.get_time()
            
            rospy.loginfo("Unknown object detected, recognizing...")

            go_forward(False)
            follow_wall(False)

            object_angle=math.atan2(detected_object.y,detected_object.x)
            object_angle=180*(object_angle/math.pi)

            if math.fabs(object_angle) > 10.0:
               turn(object_angle)
            
            rospy.sleep(RECOGNITION_TIME)

            if math.fabs(object_angle) > 10.0:
                turn(-object_angle)

            if rospy.get_time() - object_recognized_time <= RECOGNITION_TIME:
                rospy.loginfo("Reognition successful")
                place_node(True)
            else: 
                rospy.loginfo("Reognition failed")
        else:
            rospy.loginfo("Reognition successful")
            place_node(True)

        object_detected = False
        recognition_done_time = rospy.get_time()
        reset_node_detected()

        rospy.loginfo("OBJECT_DETECTED ==> EXPLORE")
        return 'explore'

class FollowGraph(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore', 'follow_graph'], 
                                    input_keys=['trait'],
                                    output_keys=['trait'])
    def execute(self, userdata):
        go_forward(False)
        follow_wall(False)
        rospy.loginfo("Disabling mapping.")
        mapping_active(False)

        rospy.loginfo("Following path.")
        path = next_noi_service.call(NextNodeOfInterestRequest(current_node.id_this, userdata.trait)).path
        follow_path_pub.publish(path)

        goto_done[0] = False
        wait_for_flag(goto_done)
        rospy.loginfo("Follow path done.")

        rospy.loginfo("Enabling mapping.")
        mapping_active(True)
        reset_node_detected()

        rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
        return 'follow_graph'


def place_node(object_here):
    global current_node, walls_have_changed
    walls_have_changed = False
    n = ObstacleHandler.north_blocked()
    e = ObstacleHandler.east_blocked()
    s = ObstacleHandler.south_blocked()
    w = ObstacleHandler.west_blocked()
    response = place_node_service.call(PlaceNodeRequest(current_node.id_this, compass_direction, n, e, s, w, object_here, detected_object.type, detected_object.x, detected_object.y))
    if not object_here:
        current_node = response.generated_node
    rospy.loginfo("Placed node with id: %d, compass direction: %d.", current_node.id_this, compass_direction)

def reset_node_detected():
    global node_detected
    node_detected = False

def turn_left():
    rospy.loginfo("Turning left.")
    turn(90.0)
    reset_object_detection_mute()
    
def turn_right():
    rospy.loginfo("Turning right.")
    turn(-90.0)
    reset_object_detection_mute()


def turn_back():
    rospy.loginfo("Turning back.")
    turn(180.0)
    reset_object_detection_mute()

def reset_object_detection_mute():
    global recognition_done_time
    recognition_done_time = 0

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

def mapping_active(active):
    mapping_active_pub.publish(active)

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
    if not (current_node.edges[robot_to_map_dir(RobotDirections.LEFT, compass_direction)]==Node.BLOCKED) == ObstacleHandler.robot_dir_blocked(RobotDirections.LEFT):
        return
        
    if not (current_node.edges[robot_to_map_dir(RobotDirections.RIGHT, compass_direction)]==Node.BLOCKED) == ObstacleHandler.robot_dir_blocked(RobotDirections.RIGHT):
        return
    if not (current_node.edges[robot_to_map_dir(RobotDirections.FORWARD, compass_direction)]==Node.BLOCKED) == ObstacleHandler.robot_dir_blocked(RobotDirections.FORWARD):
        walls_have_changed = True

def walls_changed():
    return walls_have_changed

def goto_node(node):
    global goto_done
    goto_done[0] = False
    rospy.loginfo("Going to node: %d.", node.id_this)
    goto_node_pub.publish(node)

def is_at_intersection(): 
    if walls_changed() and not obstacle_ahead() and (can_turn_right() or can_turn_left()):
            return True
    return False

def turn_done_callback(data):
    global turn_done
    turn_done[0] = True
    rospy.loginfo("Turn done callback.")

def stopping_done_callback(data):
    global stop_done
    stop_done[0] = True
    rospy.loginfo("Stopping done callback.")

def ir_callback(ir_data):
    global distance
    ObstacleHandler.distance = ir_data
    distance = ir_data

def object_detected_callback(new_object):
    global detected_object, object_detected, object_recognized_time
    if rospy.get_time() - recognition_done_time > OBJECT_DETECTION_MUTE_TIME:
        rospy.loginfo("New object detected X=%f, Y=%f, TYPE=%d.", new_object.x, new_object.y, new_object.type)
        detected_object = new_object
        object_detected = True

        if new_object.type != Object.TYPE_UNKNOWN:
            object_recognized_time = rospy.get_time()
    else: 
        rospy.loginfo("New object detected, but it was ignored. X=%f, Y=%f, TYPE=%d.", new_object.x, new_object.y, new_object.type)

def on_node_callback(node):
    global current_node, node_detected
    if(current_node.id_this != node.id_this):
        rospy.loginfo("On node callback. Previous node: %d, New node: %d.", current_node.id_this, node.id_this)
        current_node = node
        node_detected = True

def goto_done_callback(success):   
    global goto_done
    goto_done[0] = success
    rospy.loginfo("Goto callback. Success: %s.", str(success))

def odometry_callback(data):
    global odometry
    odometry = data

def compass_callback(direction):
    global compass_direction
    compass_direction = direction.data
    ObstacleHandler.compass_direction = direction

def check_for_interrupt():
    if rospy.is_shutdown():
        sys.exit(0)

def main(argv):
    global turn_pub, follow_wall_pub, go_forward_pub, place_node_service, next_noi_service, current_node, goto_node_pub, mapping_active_pub, follow_path_pub
    rospy.init_node('brain')

    sm = smach.StateMachine(outcomes=['finished'])
    rospy.Subscriber("/perception/ir/distance", Distance, ir_callback)
    rospy.Subscriber("/controller/turn/done", Bool, turn_done_callback)
    rospy.Subscriber("/vision/obstacle/object", Object, object_detected_callback)
    rospy.Subscriber("/controller/forward/stopped", Bool, stopping_done_callback)
    rospy.Subscriber("/navigation/graph/on_node", Node, on_node_callback)
    rospy.Subscriber("/pose/odometry", Odometry, odometry_callback)
    rospy.Subscriber("/pose/compass", Int8, compass_callback)
    rospy.Subscriber("/controller/goto/success", Bool, goto_done_callback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=10)
    follow_wall_pub = rospy.Publisher("/controller/wall_follow/active", Bool, queue_size=10)
    go_forward_pub = rospy.Publisher("/controller/forward/active", Bool, queue_size=10)
    goto_node_pub = rospy.Publisher("/controller/goto/target_node", Node, queue_size=1)
    follow_path_pub = rospy.Publisher("/controller/goto/follow_path", Path, queue_size=1)
    mapping_active_pub = rospy.Publisher("/mapping/active", Bool, queue_size=1)

    with sm:
        smach.StateMachine.add('EXPLORE', Explore(), transitions={'explore':'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED', 'follow_graph' : 'FOLLOW_GRAPH', 
            'object_detected' : 'OBJECT_DETECTED'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetected(), transitions={'explore': 'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED'})
        smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), transitions={'explore': 'EXPLORE'})
        smach.StateMachine.add('FOLLOW_GRAPH', FollowGraph(), transitions={'explore' : 'EXPLORE',
            'follow_graph' : 'FOLLOW_GRAPH'})

    rospy.wait_for_service('/navigation/graph/place_node')
    rospy.wait_for_service('/navigation/graph/next_node_of_interest')
    place_node_service = rospy.ServiceProxy('/navigation/graph/place_node', navigation_msgs.srv.PlaceNode)
    next_noi_service = rospy.ServiceProxy('/navigation/graph/next_node_of_interest', navigation_msgs.srv.NextNodeOfInterest)

    rospy.sleep(3.0)

    n = ObstacleHandler.north_blocked()
    e = ObstacleHandler.east_blocked()
    s = ObstacleHandler.south_blocked()
    w = True
    response = place_node_service.call(PlaceNodeRequest(-1, Node.EAST, n, e, s, w, False, -1, -1, -1))
    current_node = response.generated_node

    outcome = sm.execute() 

#if __name__ == '__main__':
main(sys.argv)

