#include <ros/ros.h>
#include <navigation_msgs/Node.h>
#include <navigation_msgs/PlaceNode.h>
#include <navigation_msgs/NextNodeOfInterest.h>
#include <nav_msgs/Odometry.h>
#include <navigation/Graph.h>
#include <navigation/GraphViz.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#define NODE_TRAIT_UNKNOWN 0
#define NODE_TRAIT_HAS_OBJECT 1

geometry_msgs::Point _position;
Graph _graph;
boost::shared_ptr<GraphViz> _graph_viz;
tf::StampedTransform _transform;

typedef struct GraphPath_t {
    std::vector<int> path;
    int next;
    int trait;
} GraphPath;
GraphPath _path;

ros::Publisher _pub_on_node;

void callback_odometry(const nav_msgs::OdometryConstPtr& odom) {

    _position = odom->pose.pose.position;
}

bool update_transform()
{
    static tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform("map", "robot", ros::Time(0), ros::Duration(1.0) );
        tf_listener.lookupTransform("map", "robot", ros::Time(0), _transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    return true;
}

bool robotToMapTransform(float x, float y, float& map_x, float& map_y)
{
    if(!update_transform()) return false;

    geometry_msgs::PointStamped robot_point_msg;
    robot_point_msg.header.frame_id = "robot";
    robot_point_msg.header.stamp = ros::Time::now();
    robot_point_msg.point.x = x;
    robot_point_msg.point.y = y;
    robot_point_msg.point.z = 0.0;
    tf::Stamped<tf::Point> robot_point;
    tf::pointStampedMsgToTF(robot_point_msg, robot_point);
    tf::Vector3 map_point = _transform*robot_point;

    map_x = map_point.x();
    map_y = map_point.y();

    return true;
}

bool service_place_node(navigation_msgs::PlaceNodeRequest& request,
                        navigation_msgs::PlaceNodeResponse& response)
{
    if (request.id_previous == -1 && _graph.num_nodes() > 0) {
        ROS_ERROR("Every node has to have a predecessor (except the first)");
        return false;
    }

    bool success = true;

    float x = _position.x;
    float y = _position.y;

    //if(robotToMapTransform(x,y, x,y))
    {

        response.generated_node = _graph.place_node(x, y, request);
        if (request.id_previous >= 0) {
            navigation_msgs::Node prev_node = _graph.get_node(request.id_previous);
            ROS_INFO("Place node. %d(%.2f,%.2f) -> %d(%.2f,%.2f)", request.id_previous, prev_node.x,prev_node.y, response.generated_node.id_this, response.generated_node.x, response.generated_node.y);
        }

        if (request.object_here == true) {
            if(robotToMapTransform(x,y, x,y)) {
                robotToMapTransform(x,y, x,y);
                _graph.place_object(response.generated_node.id_this, request);
            }
            else success = false;
        }

    }
    //else success = false;

    return success;
}

void init_path_to_noi(int id_from, int trait) {
    _path.next = 0;
    _path.trait = trait;
    _path.path.clear();

    if (trait == navigation_msgs::NextNodeOfInterestRequest::TRAIT_UNKNOWN_DIR) {
        ROS_INFO("Finding shortest path to next unkown location...");
        _graph.path_to_next_unknown(id_from, _path.path);
    }
    else if (trait == navigation_msgs::NextNodeOfInterestRequest::TRAIT_OBJECT) {
        ROS_INFO("Finding shortest path to next object...");
        _graph.path_to_next_object(id_from, _path.path);
    }
    else if (trait == navigation_msgs::NextNodeOfInterestRequest::TRAIT_START)
    {
        ROS_INFO("Finding shortest path to start node...");
        _graph.path_to_node(id_from, 0, _path.path);
    }
    else
        ROS_ERROR("Requested trait %d is not implemented yet.", trait);
}

bool service_next_noi(navigation_msgs::NextNodeOfInterestRequest& request,
                      navigation_msgs::NextNodeOfInterestResponse& response)
{
    if ( request.trait == NODE_TRAIT_UNKNOWN || request.trait == NODE_TRAIT_HAS_OBJECT )
    {
        //if we haven't finished an existing path: advance a step
        if (_path.next < _path.path.size())
        {
            if (request.trait != _path.trait) {
                ROS_ERROR("Trait was changed during path following. Will service request anyway.");
                init_path_to_noi(request.id_from, request.trait);
            }
            if (request.id_from != _path.path[_path.next]) {
                ROS_ERROR("Last path was not completed. Will service request anyway.");
                init_path_to_noi(request.id_from, request.trait);
            }
            else {
                _path.next++;
            }
        }
        else {
            init_path_to_noi(request.id_from, request.trait);
        }

        response.target_node = _graph.get_node(_path.path[_path.next]);
    }
    else {
        ROS_ERROR("Requested trait %d is not implemented yet.", request.trait);
        return false;
    }

    return true;
}

void test_request(int id_prev, int dir, bool blocked_n, bool blocked_e, bool blocked_s, bool blocked_w, navigation_msgs::PlaceNodeRequest& request)
{
    request.id_previous = id_prev;
    request.direction = dir;
    request.north_blocked = blocked_n;
    request.east_blocked = blocked_e;
    request.south_blocked = blocked_s;
    request.west_blocked = blocked_w;
}

void test_graph() {
    navigation_msgs::PlaceNodeRequest place;
    navigation_msgs::Node node;

    test_request(-1,-1, false, false, true, true, place);
    node = _graph.place_node(0,0,place);

    test_request(node.id_this,Graph::East,false,true,true,false,place);
    node = _graph.place_node(3,0,place);

    test_request(node.id_this,Graph::North,true,true,false,false,place);
    node = _graph.place_node(3,0.5,place);

    test_request(node.id_this,Graph::West,false,false,true,true,place);
    node = _graph.place_node(2.5,0.5,place);

    test_request(node.id_this,Graph::North,false,false,false,true,place);
    node = _graph.place_node(2.5,1.0,place);

    test_request(node.id_this,Graph::North,true,true,false,false,place);
    place.object_here = true;
    place.object_x = 2.8;
    place.object_y = 1.55;
    place.object_direction = Graph::East;
    node = _graph.place_node(2.5,1.5,place);
    _graph.place_object(node.id_this, place);

    test_request(node.id_this,Graph::West,true,false,false,true,place);
    node = _graph.place_node(0,1.5,place);

    test_request(node.id_this,Graph::South,false,false,true,true,place);
    node = _graph.place_node(0,0.1,place);

    std::vector<int> path;
    _graph.path_to_next_unknown(0,path);

    for(int i = 0; i < path.size(); ++i)
        std::cout << path[i] << " -> ";

    std::cout << std::endl;

    path.clear();
    _graph.path_to_next_object(0,path);

    for(int i = 0; i < path.size(); ++i)
        std::cout << path[i] << " -> ";

    std::cout << std::endl;

    test_request(4,Graph::East,false,true,true,false,place);
    place.object_here = true;
    place.object_x = 2.8;
    place.object_y = 1.48;
    place.object_direction = Graph::North;
    node = _graph.place_node(3,1.0,place);
    _graph.place_object(node.id_this,place);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph");

    _path.next = 0;

    bool test = false;
    /*test_graph(); test=true;*/

    ros::NodeHandle n;

    _pub_on_node = n.advertise<navigation_msgs::Node>("/navigation/graph/on_node",10);

    ros::Subscriber sub_odom = n.subscribe("/pose/odometry",10,callback_odometry);

    ros::ServiceServer srv_place_ndoe = n.advertiseService("/navigation/graph/place_node",service_place_node);
    ros::ServiceServer srv_next_noi = n.advertiseService("/navigation/graph/next_node_of_interest",service_next_noi);

    navigation_msgs::Node node;

    _graph_viz = boost::shared_ptr<GraphViz>(new GraphViz(_graph, n));

    ros::Rate rate(10.0);

    while(n.ok())
    {
        float x = _position.x;
        float y = _position.y;
        //if(robotToMapTransform(x,y, x,y))
        {

            if (_graph.on_node(x,y, node)) {
                _pub_on_node.publish(node);
                _graph_viz->highlight_node(node.id_this,true);
            }

            _graph_viz->draw();

        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
