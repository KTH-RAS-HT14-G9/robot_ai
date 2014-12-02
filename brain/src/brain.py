#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import sys
import time
from math import atan2
from math import pi
from math import fabs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
from ir_converter.msg import Distance

from navigation_msgs.msg import Node
from navigation_msgs.srv import *
from vision_msgs.msg import Object
from nav_msgs.msg import Odometry

######################## VARIABLES #########################
odometry = None
current_node = Node()
current_node.id_this = -1
recognition_time = 2.0
waiting_time = 0.001

rospy.wait_for_service('/navigation/graph/place_node')
rospy.wait_for_service('/navigation/graph/next_node_of_interest')
place_node = rospy.ServiceProxy('/navigation/graph/place_node', navigation_msgs.srv.PlaceNode)
next_node_of_interest = rospy.ServiceProxy('/navigation/graph/next_node_of_interest', navigation_msgs.srv.NextNodeOfInterest)

NORTH=0
EAST=1
SOUTH=2
WEST=3
current_direction = WEST
object_x = -1
object_y = -1
node_detected = False
reset_mc_pub = None
recognize_object_pub = None
turn_pub = None
follow_wall_pub = None
go_forward_pub = None

turn_threshold = 0.35
obstacle_threshold = 0.30
fl_ir = 0
fr_ir = 0
bl_ir = 0
br_ir = 0
l_front_ir = 0
r_front_ir = 0
turn_done = False
object_recognized = False
object_detected = False
object_location = None
following_wall = False
going_forward = False
stop_done = False
object_type=-1
object_angle=0
time_start=0
angle_record=0

######################## STATES #########################

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore','obstacle_detected', 'object_detected', 'intersection_detected', 'follow_graph'])

    def execute(self, userdata):
        fam_node_seen = node_detected
        ResetFamiliarNodeDetected()
        FollowWall(True)
        GoForward(True)
        if object_detected and not RecognizedBefore():
            rospy.loginfo("EXPLORE ==> OBJECT_DETECTED")
            return 'object_detected'
        elif fam_node_seen:
            rospy.loginfo("EXPLORE ==> node_detected")
            return 'follow_graph'
        elif ObstacleAhead():
            rospy.loginfo("EXPLORE ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'        
        elif IsAtIntersection() and CurrentNodeNotHere():
            #rospy.loginfo("EXPLORE ==> INTERSECTION_DETECTED")
            #return 'intersection_detected'
            rospy.loginfo("Is at intersection, placing node")
            PlaceNode(False)
        return 'explore'    

class ObstacleDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore','obstacle_detected'])

    def execute(self, userdata):
        #if not stop_done:
            GoForward(False)
            FollowWall(False)
          #  return 'obstacle_detected'
        #else:
            PlaceNode(False)
            if CanTurnLeft():
                TurnLeft()
            elif CanTurnRight():
                TurnRight()
            else:
                TurnBack()
            rospy.loginfo("OBSTACLE_DETECTED ==> EXPLORE")
            ResetFamiliarNodeDetected()
            return 'explore'

class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore'])

    def execute(self, userdata):

        GoForward(False)
        FollowWall(False)
        has_turned = False
        if (fabs(object_angle)>10):
           angle_record=object_angle
           Turn(object_angle)
           has_turned = True
        
        object_recognized = RecognizeObject()
        
        if has_turned:
            Turn(-angle_record)
        
        PlaceNode(object_recognized)

        rospy.loginfo("OBJECT_DETECTED ==> EXPLORE")
        ResetFamiliarNodeDetected()
        return 'explore'

class IntersectionDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore'])

    def execute(self, userdata):
        PlaceNode(False)   
        
        while IsAtIntersection():
            if ObstacleAhead():
                break
            rospy.sleep(waiting_time)

        ResetFamiliarNodeDetected()
        rospy.loginfo("INTERSECTION_DETECTED ==> EXPLORE")
        return 'explore'

class FollowGraph(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore', 'follow_graph'])

    def execute(self, userdata):
        
        next_node = GetNextNodeOfInterest()
        if(next_node.id_this == current_node.id_this):
            rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
            return 'explore'
        angle = GetAngleTo(GetDirectionTo(next_node))
        if angle != 0:
            GoForward(False)
            FollowWall(False)
            Turn(angle)

        FollowWall(True)
        GoForward(True)

        node_detected = False
        while not node_detected:
            if ObstacleAhead():
                rospy.loginfo("FOLLOW_GRAPH ==> EXPLORE")
                return 'explore'
            rospy.sleep(waiting_time)
        return 'follow_graph'

######################## FUNCTIONS #########################

def CurrentNodeNotHere():
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    dist = math.sqrt(math.pow(x-current_node.x, 2) + math.pow(y-current_node.y, 2))
    return dist > 0.1

def GetDirectionTo(node):
    if node.id_north == current_node.id_this:
        return SOUTH
    if node.id_west == current_node.id_this:
        return EAST
    if node.id_south == current_node.id_this:
        return NORTH
    return WEST

def GetAngleTo(direction):
    angle = 90.0 * ((current_direction - direction + 4) % 4)
    if angle == 270.0:
        return -90.0
    return angle


def RecognizedBefore():
    object_detected = False
    return current_node.object_here

def GetNextNodeOfInterest():
    return next_node_of_interest.call(NextNodeOfInterestRequest(current_node.id_this, 0)).target_node


def PlaceNode(object_here):
    global current_node
    #TODO: Fill interface with object information
    response = place_node.call(PlaceNodeRequest(current_node.id_this, current_direction, NorthBlocked(), EastBlocked(), SouthBlocked(), WestBlocked(), object_here, object_type, current_direction, object_x, object_y))
    current_node = response.generated_node

def NorthBlocked():
    if current_direction == NORTH:
        return ObstacleAhead()
    if current_direction == WEST:
        return not CanTurnRight()
    if current_direction == EAST: 
        return not CanTurnLeft()
    else: return False # TODO handle special case where the robot has just turned, then this is not true. Solve by looking at map

def WestBlocked():
    if current_direction == NORTH:
        return not CanTurnLeft()
    if current_direction == WEST:
        return ObstacleAhead()
    if current_direction == SOUTH: 
        return not CanTurnRight()
    else: return False

def SouthBlocked():
    if current_direction == WEST:
        return not CanTurnLeft()
    if current_direction == EAST:
        return not CanTurnRight()
    if current_direction == SOUTH: 
        return  ObstacleAhead()
    else: return False

def EastBlocked():
    if current_direction == NORTH:
        return not CanTurnRight()
    if current_direction == EAST:
        return ObstacleAhead()
    if current_direction == SOUTH: 
        return  CanTurnLeft()
    else: return False

def ResetFamiliarNodeDetected():
    node_detected = False

def CanTurnLeft():
    return True if fl_ir > turn_threshold and bl_ir > turn_threshold else False

def CanTurnRight():
    return True if fr_ir > turn_threshold and br_ir > turn_threshold else False

def ObstacleAhead():
    return True if l_front_ir < obstacle_threshold or r_front_ir < obstacle_threshold else False

def TurnLeft():
    rospy.loginfo("Turning left")
    Turn(90.0)
    
def TurnRight():
    rospy.loginfo("Turning right")
    Turn(-90.0)

def TurnBack():
    rospy.loginfo("Turning back")
    Turn(180.0)

def Turn(angle):
    global current_direction, turn_done
    current_direction = (current_direction + GetDirIncrement(angle)) % 4
    ResetMotorController()
    turn_done = False
    turn_pub.publish(angle)
    waitForTurnDone()

def GetDirIncrement(angle):
    if angle == 90.0:
        return 3
    if angle == -90.0:
        return 1
    if angle == 180.0:
        return 2
    return 0

def waitForTurnDone():
    while not turn_done:
        rospy.sleep(waiting_time)
    rospy.loginfo("Turn done")

def FollowWall(should_follow):
    global following_wall
    if (should_follow and not following_wall) or (not should_follow and following_wall):
        following_wall = not following_wall
        follow_wall_pub.publish(should_follow)
        rospy.loginfo("Following Wall: %s", str(should_follow))

def GoForward(should_go):
    global going_forward, stop_done
#    if (should_go and not going_forward) or (not should_go and going_forward):
    if should_go != going_forward:
        ResetMotorController()
        going_forward = should_go
        stop_done = False
        go_forward_pub.publish(should_go)
        rospy.loginfo("Going forward: %s", str(should_go))
        if not should_go:
            WaitForStopDone()

def WaitForStopDone():
    while not stop_done:
        rospy.sleep(waiting_time)
    else:    
        rospy.loginfo("Stopping Done")

def RecognizeObject():
    global object_recognized

    object_recognized = False
    rospy.loginfo("Start recognizing object")
    recognize_object_pub.publish(True)
    WaitForRecognitionDone(time.clock())
    object_detected = False

def WaitForRecogniztionDone(start_time):
    while time.clock() - start_time < recognition_time:
        rospy.sleep(waiting_time)
    if object_recognized:
        rospy.loginfo("Object recognized")
    else:
        rospy.loginfo("Recognition Failed")
    
def ResetMotorController():
    reset_mc_pub.publish(True)
    rospy.loginfo("Resetting MC pid")

def IsAtIntersection():
    return True if not ObstacleAhead() and (CanTurnRight() or CanTurnLeft()) else False

################### CALLBACKS #############################

def TurnDoneCallback(data):
    global turn_done
    turn_done = True
    rospy.loginfo("Turn done callback: %s", str(data))

def StoppingDoneCallback(data):
    global stop_done
    stop_done = True
    rospy.loginfo("Stopping done callback: %s", str(data))

def ObjectRecognizedCallback(data):
    global object_recognized
    object_recognized = True
    rospy.loginfo("Object Recognized: %s", str(data))

def ObjectDetectedCallback(data):
    global object_detected, object_location
    object_detected = True
    object_location = data
    rospy.loginfo("Object Detected: %s", str(data))

def IRCallback(data):
    global fl_ir, fr_ir, bl_ir, br_ir, l_front_ir, r_front_ir
    fl_ir = data.fl_side;
    fr_ir = data.fr_side;
    bl_ir = data.bl_side;
    br_ir = data.br_side;
    l_front_ir = data.l_front;
    r_front_ir = data.r_front;

def ObjectOrientationCallback(data):
    global object_angle,object_type, object_x, object_y
    object_type=data.type
    object_x=data.x
    object_y=data.y
    object_angle=atan2(object_y,object_x)
    if object_angle>(pi/2):
       object_angle=object_angle-pi
       object_angle=180*(object_angle/pi)

def OnNodeCallback(node):
    global current_node
    
    if(current_node.id_this == node.id_this):
        current_node = node
        node_detected = True

def OdometryCallback(data):
    global odometry
    odometry = data

def main():
    global turn_pub, follow_wall_pub, go_forward_pub, recognize_object_pub, reset_mc_pub
    rospy.init_node('brain')

    
    sm = smach.StateMachine(outcomes=['finished'])
    rospy.Subscriber("/perception/ir/distance", Distance, IRCallback)
    rospy.Subscriber("/controller/turn/done", Bool, TurnDoneCallback)
    rospy.Subscriber("/vision/recognition/done", String, ObjectRecognizedCallback) 
    rospy.Subscriber("/vision/detector/obstacle/distance", Float64, ObjectDetectedCallback) 
    rospy.Subscriber("/vision/obstacle/object", Object, ObjectOrientationCallback)
    rospy.Subscriber("/controller/forward/stopped", Bool, StoppingDoneCallback)
    rospy.Subscriber("/navigation/graph/on_node", Node, OnNodeCallback)
    rospy.Subscriber("/pose/odometry/", Odometry, OdometryCallback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=10)
    follow_wall_pub = rospy.Publisher("/controller/wall_follow/active", Bool, queue_size=10)
    go_forward_pub = rospy.Publisher("/controller/forward/active", Bool, queue_size=10)
    recognize_object_pub = rospy.Publisher("/vision/recognition/active", Bool, queue_size=10)
    reset_mc_pub = rospy.Publisher("controller/motor/reset", Bool, queue_size=1)

    with sm:
        smach.StateMachine.add('EXPLORE', Explore(), transitions={'explore':'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED',
            'intersection_detected' : 'INTERSECTION_DETECTED', 'follow_graph' : 'FOLLOW_GRAPH', 
            'object_detected' : 'OBJECT_DETECTED'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetected(), transitions={'explore': 'EXPLORE','obstacle_detected':'OBSTACLE_DETECTED'})
        smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), transitions={'explore': 'EXPLORE'})
        smach.StateMachine.add('INTERSECTION_DETECTED', IntersectionDetected(), transitions={'explore': 'EXPLORE'})
        smach.StateMachine.add('FOLLOW_GRAPH', FollowGraph(), transitions={'explore' : 'EXPLORE',
            'follow_graph' : 'FOLLOW_GRAPH'})

    
    rospy.sleep(3.0)
    outcome = sm.execute() 

#if __name__ == '__main__':
main()
