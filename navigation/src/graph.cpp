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
} GraphPath;
GraphPath _path;

ros::Publisher _pub_on_node;

void callback_odometry(const nav_msgs::OdometryConstPtr& odom) {

    _position = odom->pose.pose.position;
}

bool service_place_node(navigation_msgs::PlaceNodeRequest& request,
                        navigation_msgs::PlaceNodeResponse& response)
{
    if (request.id_previous == -1 && _graph.num_nodes() > 0) {
        ROS_ERROR("Every node has to have a predecessor (except the first)");
        return false;
    }

    response.generated_node = _graph.place_node(_position.x, _position.y, request);
    return true;
}

bool service_next_noi(navigation_msgs::NextNodeOfInterestRequest& request,
                      navigation_msgs::NextNodeOfInterestResponse& response)
{
    if ( request.trait == NODE_TRAIT_UNKNOWN ) {
        if (_path.next < _path.path.size()) {
            if (request.id_from != _path.path[_path.next]) {
                ROS_ERROR("Last path was not completed. Will service request anyway.");
                _path.next = 0;
                _path.path.clear();

                _graph.path_to_next_unknown(request.id_from, _path.path);
            }
            else {
                _path.next++;
            }
        }
        else {
            ROS_INFO("Finding shortest path to next unkown location...");
            _path.next = 0;
            _path.path.clear();

            _graph.path_to_next_unknown(request.id_from, _path.path);
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

    test_request(-1,-1, true, false, true, true, place);
    _graph.place_node(0,0,place);

    test_request(0,Graph::East,false,true,true,false,place);
    _graph.place_node(3,0,place);

    test_request(1,Graph::North,true,true,false,false,place);
    _graph.place_node(3,0.5,place);

    test_request(2,Graph::West,false,false,true,true,place);
    _graph.place_node(2.5,0.5,place);

    test_request(3,Graph::North,false,false,false,true,place);
    _graph.place_node(2.5,1.0,place);

    test_request(4,Graph::North,true,true,false,false,place);
    _graph.place_node(2.5,1.5,place);

    std::vector<int> path;
    _graph.path_to_next_unknown(0,path);

    for(int i = 0; i < path.size(); ++i)
        std::cout << path[i] << " -> ";

    std::cout << std::endl;
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

void robotToMapTransform(float x, float y, float& map_x, float& map_y)
{
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
        if(!test && !update_transform()) continue;

        float x = _position.x;
        float y = _position.y;
        if(!test) robotToMapTransform(x,y, x,y);

        if (_graph.on_node(x,y, node)) {
            _pub_on_node.publish(node);
            _graph_viz->highlight_node(node.id_this,true);
        }

        _graph_viz->draw();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
