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

m3_object_recognized = False

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

current_direction = MapDirections.EAST

go_to_node_pub = None
reset_mc_pub = None
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
object_reached = False

class M3Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['m3_explore','obstacle_detected', 'object_detected', 'm3_go_to_start'])

    def execute(self, userdata):

        if object_reached: 
            rospy.logerr("SUCCESS! OBJECT REACHED AGAIN")
            sys.exit(0)

        follow_wall(True)
        go_forward(True)
        update_walls_changed()

        if m3_object_recognized:
            go_forward(False)
            follow_wall(False)
            rospy.loginfo("M3EXPLORE ==> GO_TO_START")
            return 'm3_go_to_start'
        if object_detected:
            rospy.loginfo("M3EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        elif obstacle_ahead():
            rospy.loginfo("M3EXPLORE ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'        
        elif is_at_intersection():
         #   rospy.loginfo("Is at intersection, placing node")
            place_node(False)
        return 'm3_explore'

class M3GoToStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['m3_go_to_object','m3_go_to_start'])

    def execute(self, userdata):        
        rospy.logerr("BEFORE GET NOI")
        next_node = get_next_noi(2)
        rospy.logerr("AFTER GET NOI")

        rospy.logerr("current_node: %s next_node: %s", current_node, next_node)
        if(next_node.id_this == current_node.id_this):
            rospy.loginfo("Destination reached")
            rospy.loginfo("M3_GO_TO_START ==> M3_GO_TO_OBJECT")
            go_forward(False)
            follow_wall(False)
            return 'm3_go_to_object'
        
        go_to_node(next_node)

        rospy.logerr("waiting for go to node")

        wait_for_flag(go_to_node_done)

        rospy.logerr("finished waiting for go to node")
    #    angle = get_angle_to(get_direction_to(next_node))
    #    rospy.logerr("angle: %f", angle)
    #    if angle != 0.0:
    #        go_forward(False)
    #        rospy.logerr("stopped")
    #        follow_wall(False)
    #        rospy.logerr("turning...")
    #        turn(angle)
    #        rospy.loginfo("turned")

    #    follow_wall(True)
    #    go_forward(True)
    #    rospy.logerr("going forward until node or obstacle")
    
       # while not node_detected:
        #   if obstacle_ahead() or object_detected:
        #        rospy.logerr('obstacle detected in m3gotostart')
        #        break
        #    check_for_interrupt()
        #    rospy.sleep(WAITING_TIME)
        # reset_node_detected()

        return 'm3_go_to_start'

class M3GoToObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['m3_go_to_object','object_detected'])

    def execute(self, userdata):
        global object_reached
        next_node = get_next_noi(1)
        rospy.logerr("current_node: %s next_node: %s", current_node, next_node)

        if(next_node.object_here):
            rospy.loginfo("Destination reached")
            rospy.loginfo("M3_GO_TO_OBJECT ==> OBJECT_DETECTED")
            go_forward(False)
            follow_wall(False)

            turn(-90.0)
            object_reached = True
            return 'object_detected'
       
        go_to_node(next_node)
        wait_for_flag(go_to_node_done)
       # angle = get_angle_to(get_direction_to(next_node))
       # if angle != 0:
       #     go_forward(False)
       #     follow_wall(False)
       #     turn(angle)

        #follow_wall(True)
        #go_forward(True)
        
        #while not node_detected:
          #  if obstacle_ahead() or object_detected:
          #      rospy.logerr('obstacle detected in m3gotoobject')
          #      break
        #    check_for_interrupt()
        #    rospy.sleep(WAITING_TIME)

        #reset_node_detected()
        return 'm3_go_to_object'
        

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
        if object_detected and not recognized_before():
            rospy.loginfo("EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        #elif node_seen:
           # rospy.loginfo("EXPLORE ==> FOLLOW_GRAPH")
         #   return 'follow_graph'
        elif obstacle_ahead():
            rospy.loginfo("EXPLORE ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'        
        elif is_at_intersection():
            #rospy.loginfo("Is at intersection, placing node")
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
        global m3_object_recognized

        if detected_object.type == -1:

            rospy.loginfo("Unknown object seen, trying to recognize")
            go_forward(False)
            follow_wall(False)

            object_angle=math.atan2(detected_object.y,detected_object.x)
            rospy.logerr("angle to object: %f rad", object_angle)
            object_angle=180*(object_angle/math.pi)
            rospy.logerr("angle to object: %f deg", object_angle)

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
            m3_object_recognized = True

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

        while not node_detected:
            if obstacle_ahead() or object_detected:
                rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
                return 'explore'
            check_for_interrupt()
            rospy.sleep(WAITING_TIME)
        return 'follow_graph'

def check_for_interrupt():
    if rospy.is_shutdown():
        return sys.exit(0)

def obstacle_behind():
    response = fit_blob_service.call(FitBlobRequest(-ROBOT_DIAMETER+0.03, 0.0, 0.08, 0.05))
    return not response.fits

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

def get_angle_to(map_dir):
    angle = 90.0 * ((current_direction - map_dir + 4) % 4)
    if angle == 270.0:
        return -90.0
    return angle

def recognized_before():
    global object_detected
    object_detected = False
    return current_node.object_here

def get_next_noi(mode):
    return next_noi_service.call(NextNodeOfInterestRequest(current_node.id_this, mode)).target_node

def place_node(object_here):
    global current_node, walls_have_changed
    walls_have_changed = False
   # rospy.logerr("ID: %d C: %d N: %s, E: %s, S: %s, W: %s", current_node.id_this, current_direction, north_blocked(), east_blocked(), south_blocked(), west_blocked())
    response = place_node_service.call(PlaceNodeRequest(current_node.id_this, current_direction, north_blocked(), east_blocked(), south_blocked(), west_blocked(), object_here, detected_object.type, current_direction, detected_object.x, detected_object.y))
    rospy.logerr("PLACED NODE: %d, CURRENT: %d", response.generated_node.id_this, current_node.id_this)
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

def reset_node_detected():
    global node_detected
    node_detected = False

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
    if math.fabs(angle) < 1.0:
        return
    reset_motor_controller()
    turn_done[0] = False
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
    while not flag[0]:
        check_for_interrupt()
        rospy.sleep(WAITING_TIME)

def follow_wall(should_follow):
    global following_wall
    if should_follow != following_wall:
        following_wall = should_follow
        follow_wall_pub.publish(should_follow)
        rospy.loginfo("Following Wall: %s", str(should_follow))

def go_forward(should_go):
    global going_forward, stop_done

    if should_go != going_forward:
        reset_motor_controller()
        going_forward = should_go
        stop_done[0] = False
        go_forward_pub.publish(should_go)
        rospy.loginfo("Going forward: %s", str(should_go))
        if not should_go:
            wait_for_flag(stop_done)
            rospy.loginfo("Stopping Done")

def reset_motor_controller():
    reset_mc_pub.publish(True)
    rospy.loginfo("Resetting MC pid")

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
    rospy.logerr("GO TO NODE: %d", node.id_this)
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
    if current_direction == MapDirections.NORTH:
        return robot_dir
    if current_direction == MapDirections.EAST:
        return (robot_dir + 1) % 4 
    if current_direction == MapDirections.SOUTH:
        return (robot_dir + 2) % 4
    if current_direction == MapDirections.WEST:
        return (robot_dir + 3) % 4

def map_to_robot_dir(map_dir):
    if current_direction == MapDirections.NORTH:
        return map_dir
    if current_direction == MapDirections.WEST:
        return (map_dir + 1) % 4
    if current_direction == MapDirections.SOUTH:
        return (map_dir + 2) % 4
    if current_direction == MapDirections.EAST:
        return (map_dir + 3) % 4

def turn_done_callback(data):
    global turn_done
    turn_done[0] = True
    rospy.loginfo("turn done callback: %s", str(data))

def stopping_done_callback(data):
    global stop_done
    stop_done[0] = True
    rospy.loginfo("Stopping done callback: %s", str(data))

def ir_callback(data):
    global distance
    distance = data

def object_detected_callback(new_object):
    global detected_object, object_detected, recognition_clock
   # if distance_between(new_object, detected_object) > 0.2 or (new_object.type != -1 and new_object.type != object_detected.type):
    rospy.logerr("NEW OBJECT X=%f, Y=%f, TYPE=%d", new_object.x, new_object.y, new_object.type)
    rospy.loginfo("Object detected")
    
    if new_object.type != -1:
        detected_object = new_object
        object_detected = True
    #if detected_object.type != -1:
    #    recognition_clock = rospy.get_time()

def distance_between(object1, object2):
    return math.sqrt(math.pow(object1.x-object2.x,2)+math.pow(object1.y-object2.y,2))

def on_node_callback(node):
    global current_node, node_detected
    if(current_node.id_this != node.id_this):
        rospy.logerr("NODE. Current: %d, detected: %d", current_node.id_this, node.id_this)
        current_node = node
        node_detected = True

def go_to_node_done_callback(success):   
    global go_to_node_done
    go_to_node_done[0] = success
    rospy.loginfo("Go to node callback: %s", str(success))

def odometry_callback(data):
    global odometry
    odometry = data

def main(argv):
    global turn_pub, follow_wall_pub, go_forward_pub, recognize_object_pub, reset_mc_pub, place_node_service, next_noi_service, current_node, fit_blob_service, go_to_node_pub#, fetch_objects
    rospy.init_node('brain')

    #if len(argv) > 1 and argv[1] == 'fetch':
    #    fetch_objects = True

    sm = smach.StateMachine(outcomes=['finished'])
    rospy.Subscriber("/perception/ir/distance", Distance, ir_callback)
    rospy.Subscriber("/controller/turn/done", Bool, turn_done_callback)
    rospy.Subscriber("/vision/obstacle/object", Object, object_detected_callback)
    rospy.Subscriber("/controller/forward/stopped", Bool, stopping_done_callback)
    rospy.Subscriber("/navigation/graph/on_node", Node, on_node_callback)
    rospy.Subscriber("/pose/odometry/", Odometry, odometry_callback)
    rospy.Subscriber("/controller/goto/success", Bool, go_to_node_done_callback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=10)
    follow_wall_pub = rospy.Publisher("/controller/wall_follow/active", Bool, queue_size=10)
    go_forward_pub = rospy.Publisher("/controller/forward/active", Bool, queue_size=10)
    reset_mc_pub = rospy.Publisher("controller/motor/reset", Bool, queue_size=1)
    go_to_node_pub = rospy.Publisher("controller/goto/target_node", Node, queue_size=1)

    with sm:
        smach.StateMachine.add('EXPLORE', Explore(), transitions={'explore':'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED', 'follow_graph' : 'FOLLOW_GRAPH', 
            'object_detected' : 'OBJECT_DETECTED', 'm3_explore': 'M3_EXPLORE'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetected(), transitions={'explore': 'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED'})
        smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), transitions={'explore': 'EXPLORE'})
        smach.StateMachine.add('FOLLOW_GRAPH', FollowGraph(), transitions={'explore' : 'EXPLORE',
            'follow_graph' : 'FOLLOW_GRAPH'})

        smach.StateMachine.add('M3_EXPLORE', M3Explore(), transitions={'m3_explore' :'M3_EXPLORE', 'm3_go_to_start' : 'M3_GO_TO_START', 'object_detected':'OBJECT_DETECTED', 'obstacle_detected':'OBSTACLE_DETECTED'})
        smach.StateMachine.add('M3_GO_TO_START', M3GoToStart(), transitions={'m3_go_to_start' :'M3_GO_TO_START', 'm3_go_to_object' : 'M3_GO_TO_OBJECT'})
        smach.StateMachine.add('M3_GO_TO_OBJECT', M3GoToObject(), transitions={'m3_go_to_object' :'M3_GO_TO_OBJECT', 'object_detected' : 'OBJECT_DETECTED'})

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

