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

import direction_handler
import obstacle_handler

RECOGNITION_TIME = 2.0
WAITING_TIME = 0.005

recognition_clock = 0.0
detected_object = Object()
odometry = Odometry()
distance = Distance()
current_node = Node()

compass_direction = Node.EAST

go_to_node_pub = None
turn_pub = None
follow_wall_pub = None
go_forward_pub = None
place_node_service = None
next_noi_service = None

turn_done = [False] # In array so it can be passed by reference
go_to_node_done = [False]
stop_done = [False]
object_detected = False
following_wall = False
going_forward = False
walls_have_changed = True
node_detected = False

# TODO - what happens when there are no objects to follow in graph? How distinguish visited from unvisited objects?

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=
                                    ['explore','obstacle_detected', 
                                    'object_detected', 'follow_graph'], 
                                    output_keys=['target_trait'])

    def execute(self, userdata):
        check_for_interrupt()
        follow_wall(True)
        go_forward(True)
        update_walls_changed()
        if object_detected:
            rospy.loginfo("EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        elif node_detected:
            reset_node_detected()
            go_forward(False)
            follow_wall(False)
            userdata.target_trait = NextNodeOfInterestRequest.TRAIT_UNKNOWN_DIR
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
            if can_turn_left(distance):
                turn_left()
            elif can_turn_right(distance):
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

            rospy.loginfo("Unknown object seen, recognizing...")

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
        smach.State.__init__(self, outcomes=['explore', 'follow_graph'], 
                                    input_keys=['target_trait'],
                                    output_keys=['target_trait'])

    def execute(self, userdata):
        next_node = get_next_noi(userdata.target_trait)
        if(next_node.id_this == current_node.id_this):
            go_forward(False)
            follow_wall(False)
            rospy.loginfo("Destination reached")
            rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
            return 'explore'
        
        go_to_node(next_node)
        wait_for_flag(go_to_node_done)

        return 'follow_graph'

def get_next_noi(mode):
    return next_noi_service.call(NextNodeOfInterestRequest(current_node.id_this, mode)).target_node

def place_node(object_here):
    global current_node, walls_have_changed
    walls_have_changed = False
    response = place_node_service.call(PlaceNodeRequest(current_node.id_this, compass_direction, north_blocked(), east_blocked(), south_blocked(), west_blocked(), object_here, detected_object.type, detected_object.x, detected_object.y))
    if not object_here:
        current_node = response.generated_node

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
    if not ((current_node.edges[Node.NORTH]==Node.BLOCKED) == map_dir_blocked(Node.NORTH) and 
            (current_node.edges[Node.EAST] ==Node.BLOCKED) == map_dir_blocked(Node.EAST) and
            (current_node.edges[Node.SOUTH]==Node.BLOCKED) == map_dir_blocked(Node.SOUTH) and 
            (current_node.edges[Node.WEST] ==Node.BLOCKED) == map_dir_blocked(Node.WEST)):
        walls_have_changed = True

def walls_changed():
    return walls_have_changed

def go_to_node(node):
    global go_to_node_done
    go_to_node_done[0] = False
    rospy.loginfo("Going to node: %d.", node.id_this)
    go_to_node_pub.publish(node)

def is_at_intersection(): 
    if walls_changed() and not obstacle_ahead() and (can_turn_right(distance) or can_turn_left(distance)):
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

def ir_callback(data):
    global distance
    distance = data

def object_detected_callback(new_object):
    global detected_object, object_detected, recognition_clock
    #local coordinates, do in global
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
        rospy.loginfo("On node callback. Previous node: %d, New node: %d.", current_node.id_this, node.id_this)
        current_node = node
        node_detected = True

def go_to_node_done_callback(success):   
    global go_to_node_done
    go_to_node_done[0] = success
    rospy.loginfo("Go to node callback. Success: %s.", str(success))

def odometry_callback(data):
    global odometry
    odometry = data

def compass_callback(data):
    global compass_direction
    compass_direction = data

def check_for_interrupt():
    if rospy.is_shutdown():
        sys.exit(0)

def main(argv):
    global turn_pub, follow_wall_pub, go_forward_pub, place_node_service, next_noi_service, current_node, go_to_node_pub
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

    response = place_node_service.call(PlaceNodeRequest(-1, Node.EAST, north_blocked(), east_blocked(), south_blocked(), True, False, -1, -1, -1, -1))
    current_node = response.generated_node

    outcome = sm.execute() 

#if __name__ == '__main__':
main(sys.argv)

