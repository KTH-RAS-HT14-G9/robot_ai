from enum import IntEnum
from navigation_msgs.msg import Node

class RobotDirections(IntEnum):
    FORWARD=0
    RIGHT=1
    BACKWARD=2
    LEFT=3 

def robot_to_map_dir(robot_dir):
    if compass_direction == Node.NORTH:
        return robot_dir
    if compass_direction == Node.EAST:
        return (robot_dir + 1) % 4 
    if compass_direction == Node.SOUTH:
        return (robot_dir + 2) % 4
    if compass_direction == Node.WEST:
        return (robot_dir + 3) % 4

def map_to_robot_dir(map_dir):
    if compass_direction == Node.NORTH:
        return map_dir
    if compass_direction == Node.WEST:
        return (map_dir + 1) % 4
    if compass_direction == Node.SOUTH:
        return (map_dir + 2) % 4
    if compass_direction == Node.EAST:
        return (map_dir + 3) % 4

def get_direction_to(node):
    if node.edges[Node.NORTH] == current_node.id_this:
        return Node.SOUTH
    if node.edges[Node.EAST] == current_node.id_this:
        return Node.WEST
    if node.edges[Node.SOUTH] == current_node.id_this:
        return Node.NORTH
    if node.edges[Node.WEST] == current_node.id_this:
        return Node.EAST
    rospy.logerr("Next node is not neighbour to this node. current: %s, goal: %s", str(current_node), str(node))