#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import sys
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from ir_converter.msg import Distance

######################## VARIABLES #########################

reset_mc_pub = None
recognize_object_pub = None
turn_pub = None
follow_wall_pub = None
go_forward_pub = None

turn_threshold = 0.35
obstacle_threshold = 0.20
fl_side = 0
fr_side = 0
bl_side = 0
br_side = 0
l_front = 0
r_front = 0
turn_done = False
recognizing_done = False
object_detected = False
object_location = None
following_wall = False
go_forward = False
stopping_done = False


######################## STATES #########################

# define state GoForward
class GoForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_forward', 'stopping'])

    def execute(self, userdata):
        
        if ObstacleAhead():
            StopGoForward()
            StopFollowWall()
            rospy.loginfo("GOING_FORWARD ==> STOPPING")
            return 'stopping'
        else:
            StartFollowWall()
            StartGoForward()
            return 'go_forward'

# define state Stopping
class Stopping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopping', 'obstacle_detected', 'object_detected'])

    def execute(self, userdata):
        global stopping_done
        
        if stopping_done:
            stopping_done = False
            if object_detected:
                rospy.loginfo("STOPPING ==> OBJECT_DETECTED")
                return 'object_detected'
            else:
                rospy.loginfo("STOPPING ==> OBSTACLE_DETECTED")
                return 'obstacle_detected'
        else:
            return 'stopping'

# define state ObstacleDetected
class ObstacleDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning'])

    def execute(self, userdata):
 
        if CanTurnLeft():
            TurnLeft()
        elif CanTurnRight():
            TurnRight()
        else:
            TurnBack()
        rospy.loginfo("OBSTACLE_DETECTED ==> TURNING")
        return 'turning'

# define state ObjectDetected
class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognizing'])

    def execute(self, userdata):
       
        RecognizeObject()
        rospy.loginfo("OBJECT_DETECTED ==> RECOGNIZING")
        return 'recognizing'

 # define state Turning
class Turning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning','go_forward'])

    def execute(self, userdata):
        global turn_done
 
        if turn_done:
            turn_done = False
            rospy.set_param('/controller/wall_follow/kp', 10.0)
            StartFollowWall()
            rospy.sleep(1.0)
            rospy.set_param('/controller/wall_follow/kp', 4.0)
            rospy.loginfo("TURNING ==> GOING_FORWARD")
            return 'go_forward'       
        else:
            return 'turning'

 # define state Recognizing
class RecognizingObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognizing','obstacle_detected'])

    def execute(self, userdata):
        global recognizing_done, object_detected
        
        if recognizing_done:
            recognizing_done = False
            object_detected = False
            rospy.loginfo("RECOGNIZING ==> OBSTACLE_DETECTED")
            return 'obstacle_detected'       
        else:
            return 'recognizing'

######################## FUNCTIONS #########################

def CanTurnLeft():
    return True if fl_side > turn_threshold and bl_side > turn_threshold else False

def CanTurnRight():
    return True if fr_side > turn_threshold and br_side > turn_threshold else False

def ObstacleAhead():
    return True if l_front < obstacle_threshold or r_front < obstacle_threshold else False

def TurnLeft():
    ResetMC()
    turn_pub.publish(90.0)
    rospy.loginfo("Turning left")

def TurnRight():
    ResetMC()
    turn_pub.publish(-90.0)
    rospy.loginfo("Turning right")

def TurnBack():
    ResetMC()
    turn_pub.publish(180.0)
    rospy.loginfo("Turning back")

def StartFollowWall():
    global following_wall
    if not following_wall:
        ResetMC()
        following_wall = True
        follow_wall_pub.publish(True)
        rospy.loginfo("Start Following Wall")

def StopFollowWall():
    global following_wall
    if following_wall:
        ResetMC()
        following_wall = False
        follow_wall_pub.publish(False)
        rospy.loginfo("Stop Following Wall")

def StartGoForward():
    global go_forward
    if not go_forward:
        ResetMC()
        go_forward = True
        go_forward_pub.publish(True)
        rospy.loginfo("Start going forward")

def StopGoForward():
    global go_forward
    if go_forward:
        ResetMC()
        go_forward = False
        go_forward_pub.publish(False)
        rospy.loginfo("Stop going forward")

def RecognizeObject():
    recognize_object_pub.publish(True)
    rospy.loginfo("Start recognizing object")

def ResetMC():
    reset_mc_pub.publish(True)
    rospy.loginfo("Resetting MC pid")

def TurnDoneCallback(data):
    global turn_done
    turn_done = True
    rospy.loginfo("Turn done callback: %s", str(data))

def StoppingDoneCallback(data):
    global stopping_done
    stopping_done = True
    rospy.loginfo("Stopping done callback: %s", str(data))

def ObjectRecognizedCallback(data):
    global recognizing_done
    recognizing_done = data
    rospy.loginfo("Object Recognized: %s", str(data))

def ObjectDetectedCallback(data):
    global object_detected, object_location
    object_detected = True
    object_location = data
    rospy.loginfo("Object Detected: %s", str(data))

def IRCallback(data):
    global fl_side, fr_side, bl_side, br_side, l_front, r_front
    fl_side = data.fl_side;
    fr_side = data.fr_side;
    bl_side = data.bl_side;
    br_side = data.br_side;
    l_front = data.l_front;
    r_front = data.r_front;

def main():
    global turn_pub, follow_wall_pub, go_forward_pub, recognize_object_pub, reset_mc_pub
    rospy.init_node('brain')
    
    sm = smach.StateMachine(outcomes=['error'])
    rospy.Subscriber("/robot_ai/distance", Distance, IRCallback)
    rospy.Subscriber("/controller/turn/done", Bool, TurnDoneCallback)
    rospy.Subscriber("/vision/recognition/done", String, ObjectRecognizedCallback) 
    rospy.Subscriber("/vision/detector/obstacle/distance", Float64, ObjectDetectedCallback) 
    rospy.Subscriber("/controller/forward/stopped", Bool, StoppingDoneCallback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=10)
    follow_wall_pub = rospy.Publisher("/controller/wall_follow/active", Bool, queue_size=10)
    go_forward_pub = rospy.Publisher("/controller/forward/active", Bool, queue_size=10)
    recognize_object_pub = rospy.Publisher("/vision/recognition/active", Bool, queue_size=10)
    reset_mc_pub = rospy.Publisher("controller/motor/reset", Bool, queue_size=1)

    with sm:
        smach.StateMachine.add('GO_FORWARD', GoForward(), 
                               transitions={'go_forward':'GO_FORWARD',
                               'stopping':'STOPPING'})
        smach.StateMachine.add('STOPPING', Stopping(), 
                               transitions={'stopping':'STOPPING',
                               'obstacle_detected':'OBSTACLE_DETECTED',
                               'object_detected':'OBJECT_DETECTED'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetected(), 
                               transitions={'turning':'TURNING'})
        smach.StateMachine.add('TURNING', Turning(), 
                               transitions={'turning':'TURNING',
                               'go_forward':'GO_FORWARD'})
        smach.StateMachine.add('OBJECT_DETECTED', ObjectDetected(), 
                               transitions={'recognizing':'RECOGNIZING'})
        smach.StateMachine.add('RECOGNIZING', RecognizingObject(), 
                               transitions={'recognizing':'RECOGNIZING',
                               'obstacle_detected':'OBSTACLE_DETECTED'})

    # sleep to avoid that a decision is made before sensor data is received
    rospy.sleep(1.) 

    outcome = sm.execute()
    #rospy.spin() 

if __name__ == '__main__':
    main()
