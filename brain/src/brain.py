#!/usr/bin/env python
import roslib#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import sys
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from ras_arduino_msgs.msg import ADConverter

######################## VARIABLES #########################

fl_side = 0
fr_side = 0
bl_side = 0
br_side = 0
l_front = 0
r_front = 0
recognize_object_pub = None
turn_pub = None
follow_wall_pub = None
turn_done = False
object_recognized = False

######################## STATES #########################

# define state GoForward
class GoForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_forward', 'obstacle_detected', 'object_detected'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GO_FORWARD')
        
        if ObstacleAhead():
            return 'obstacle_detected'
        return 'go_forward'

# define state ObstacleDetected
class ObstacleDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ObstacleDetected')
        if CanTurnLeft():
            TurnLeft()
        if CanTurnRight():
            TurnRight()
        else:
            TurnBack()
        return 'turning'

# define state ObjectDetected
class ObjectDetected(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognizing'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ObjectDetected')
        RecognizeObject()
        return 'recognizing'

 # define state Turning
class Turning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning','go_forward'])

    def execute(self, userdata):
        global turn_done
        rospy.loginfo('Executing state Turning')
        if turn_done:
            turn_done = False
            return 'go_forward'       
        else:
            return 'turning'

 # define state Recognizing
class RecognizingObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognizing','obstacle_detected'])

    def execute(self, userdata):
        global recognizing
        rospy.loginfo('Executing state Recognizing')
        if recognizing:
            recognizing = False
            return 'obstacle_detected'       
        else:
            return 'recognizing'

######################## FUNCTIONS #########################

def CanTurnLeft():
    return True if fl_side > 20 and bl_side > 20 else False

def CanTurnRight():
    return True if fr_side > 20 and br_side > 20 else False

def ObstacleAhead():
    return True if l_front < 15 and r_front < 15 else False

def TurnLeft():
    turn_pub.publish(90.0)
    rospy.loginfo("Turning left")

def TurnRight():
    turn_pub.publish(-90.0)
    rospy.loginfo("Turning right")

def TurnBack():
    turn_pub.publish(180.0)
    rospy.loginfo("Turning back")

def FollowWall():
    follow_wall_pub.publish(True)
    rospy.loginfo("Start Following Wall")

def StopFollowWall():
    follow_wall_pub.publish(False)
    rospy.loginfo("Stop Following Wall")

def RecognizeObject():
    recognize_object_pub.publish(True)

def TurnDoneCallback(data):
    global turn_done
    turn_done = data
    rospy.loginfo("Turn done callback: %s", str(data))

def ObjectRecognizedCallback(data):
    global object_recognized
    object_recognized = data
    rospy.loginfo("Object Recognized: %s", str(data))

def IRCallback(data):
    global fl_side, fr_side, bl_side, br_side, l_front, r_front
    fl_side = data.ch1;
    fr_side = data.ch2;
    bl_side = data.ch3;
    br_side = data.ch4;
    l_front = data.ch7;
    r_front = data.ch8;
    rospy.loginfo("IR callback: %d, %d, %d, %d, %d, %d", data.ch1, data.ch2, data.ch3 ,data.ch4 ,data.ch7,data.ch8)
    rospy.loginfo("vars: %d, %d", fl_side, fr_side)

def main():
    global turn_pub, follow_wall_pub, recognize_object_pub
    rospy.init_node('brain')
    
    sm = smach.StateMachine(outcomes=['error'])
    rospy.Subscriber("/arduino/adc", ADConverter, IRCallback)
    rospy.Subscriber("/controller/turn/done", Bool, TurnDoneCallback)
    rospy.Subscriber("/vision/object_recognized", String, ObjectRecognizedCallback)

    turn_pub = rospy.Publisher("/controller/turn/angle", Float64, queue_size=1)
    follow_wall_pub = rospy.Publisher("/controller/follow_wall/activate", Bool, queue_size=1)
    recognize_object_pub = rospy.Publisher("/vision/recognize_object", Bool, queue_size=1)
    with sm:
        smach.StateMachine.add('GO_FORWARD', GoForward(), 
                               transitions={'go_forward':'GO_FORWARD',
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