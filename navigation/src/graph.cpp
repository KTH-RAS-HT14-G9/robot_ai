#include <ros/ros.h>
#include <navigation_msgs/Node.h>
#include <navigation_msgs/PlaceNode.h>
#include <navigation_msgs/NextNodeOfInterest.h>
#include <nav_msgs/Odometry.h>
#include <navigation/Graph.h>
#include <navigation/GraphViz.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <algorithm>
#include <vector>
#include <iostream>


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

    response.generated_node = _graph.place_node(x, y, request);
    if (request.id_previous >= 0) {
        navigation_msgs::Node prev_node = _graph.get_node(request.id_previous);
        ROS_INFO("Place node. %d(%.2f,%.2f) -> %d(%.2f,%.2f)", request.id_previous, prev_node.x,prev_node.y, response.generated_node.id_this, response.generated_node.x, response.generated_node.y);
    }

    if (request.object_here == true) {

      //  if(robotToMapTransform(request.object_x,request.object_y, request.object_x,request.object_y))
        {
            _graph.place_object(response.generated_node.id_this, request);
        }
      //else success = false;
    }

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
        double dummy;
        _graph.path_to_node(id_from, 0, _path.path,dummy);
    }
    else
        ROS_ERROR("Requested trait %d is not implemented yet.", trait);

//    std::cout << "Path: ";
//    for(int i = 0; i < _path.path.size(); ++i) {
//        std::cout << _path.path[i] << " ";
//    }
//    std::cout << std::endl;
}

bool service_next_noi(navigation_msgs::NextNodeOfInterestRequest& request,
                      navigation_msgs::NextNodeOfInterestResponse& response)
{
    if ( request.trait == navigation_msgs::NextNodeOfInterestRequest::TRAIT_UNKNOWN_DIR ||
         request.trait == navigation_msgs::NextNodeOfInterestRequest::TRAIT_OBJECT ||
         request.trait == navigation_msgs::NextNodeOfInterestRequest::TRAIT_START )
    {
//        //if we haven't finished an existing path: advance a step
//        if (_path.next < _path.path.size())
//        {
//            if (request.trait != _path.trait) {
//                ROS_ERROR("Trait was changed during path following. Will service request anyway.");
//                init_path_to_noi(request.id_from, request.trait);
//            }
//            if (request.id_from != _path.path[_path.next]) {
//                ROS_ERROR("Last path was not completed. Will service request anyway.");
//                init_path_to_noi(request.id_from, request.trait);
//            }
//        }
//        else {
//            init_path_to_noi(request.id_from, request.trait);
//        }

        init_path_to_noi(request.id_from, request.trait);

        response.path.path.clear();
        response.path.path.clear();
        response.path.path.reserve(_path.path.size());
        for(int i = 1; i < _path.path.size(); ++i) {
            response.path.path.push_back(_graph.get_node(_path.path[i]));
        }
    }
    else {
        ROS_ERROR("Requested trait %d is not implemented yet.", request.trait);
        return false;
    }

    return true;
}



int factorial_cal(int n)
{
 if (n==0) return 1;
 int fact=1;
 for (int i=1;i<=n;++i)
 {
    fact=fact*i;
 }
 return fact;
}

void get_object_node_indexs(std::vector<int> &object_nodes)
{
    object_nodes.reserve(_graph.num_nodes());


    for (int i=0; i< _graph.num_nodes();++i)
    {
        if (_graph.get_node(i).object_here)
        {
           object_nodes.push_back(_graph.get_node(i).id_this);
        }
    }
   // std::cout<<object_nodes.size()<<std::endl;
}

void generate_all_permutations( std::vector<int>& object_nodes,std::vector<std::vector <int> >& perm)
{
    int i=0;
    do {
        perm[i][0]=_graph.get_node(0).id_this; // add start node
        for (int j=0;j<object_nodes.size();++j)
        {
            perm[i][j+1]=object_nodes[j];
        }
        i=i+1;
      } while ( std::next_permutation(object_nodes.begin(),object_nodes.end() ));
}

std::vector<int> find_shortest_path()
{

    std::vector<int> object_nodes;
    std::vector<int> best_path;
    get_object_node_indexs(object_nodes);
    int perm_num=factorial_cal(object_nodes.size());
    if (object_nodes.size()==0) {best_path.push_back(_graph.get_node(0).id_this);}

    std::vector<std::vector <int> > perm;
    perm.resize(perm_num);
    best_path.reserve(object_nodes.size()+2);
    for (int i=0;i<perm_num;++i)
    {
        perm[i].resize(object_nodes.size()+1); //+1 for starting node
    }

    generate_all_permutations(object_nodes,perm);
    double shortest=std::numeric_limits<double>::infinity();
    int best_id =-1;
    double dist=0;
    // do iterations for all permutations
    for (int i=0; i<perm_num;++i)
    {
     double dist_sum=0;
     for (int j=0; j<object_nodes.size();++j)
       {
         _graph.path_to_node(perm[i][j], perm[i][j+1], _path.path,dist);
         std::cout<<"from "<<perm[i][j]<<"to "<<perm[i][j+1]<<" dist"<<dist<<std::endl;
        dist_sum=dist_sum+dist;
       }

      _graph.path_to_node(perm[i][object_nodes.size()-1], perm[i][0], _path.path,dist); // for the last object to the strating point
      std::cout<<"going back dis  "<<dist<< std::endl;
      dist_sum=dist_sum+dist;
      std::cout<<dist_sum<<std::endl;
      if (dist_sum<shortest)
      {
          best_id=i;
          shortest=dist_sum;
      }
    }
    if (best_id== -1)
    {
        std::cout<<"china"<<std::endl;
        best_path.push_back(_graph.get_node(0).id_this);
     }
    for (int i=0; i<object_nodes.size()+1;++i)
    {

        best_path.push_back(perm[best_id][i]);
    }
        best_path.push_back(_graph.get_node(0).id_this);
    return best_path;

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

class Point {
public:
    Point(double xv,double yv)
        :x(xv),y(yv){}
    double x,y;
    int dir;
};

void linspace(std::vector<Point>& target, Point from, Point to, int dir, int steps)
{
    boost::mt19937 rng;
    boost::normal_distribution<> nd(0.0, 0.1);
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > var_nor(rng,nd);

    Point cur = from;
    cur.dir = dir;
    double incX = (to.x-from.x)/steps;
    double incY = (to.y-from.y)/steps;
    for(int i = 0; i < steps; ++i)
    {
        target.push_back(cur);
        cur.x += incX + var_nor();
        cur.y += incY + var_nor();
    }

    target.push_back(cur);
}

int _test2_i = 0;
navigation_msgs::Node _test2_previous_node;

void test2_graph_build(std::vector<Point>& points) {

    Point p0(0,0);
    Point p1(2,0);
    //Point pObject1(2,0.5); pObject1.dir = navigation_msgs::Node::OBJECT;
    Point p2(2,1);
    Point p3(2,-1);
    //Point pObject2(2,-1.5); pObject2.dir = navigation_msgs::Node::OBJECT;

    linspace(points, p0, p1, navigation_msgs::Node::EAST, 5);
    p1 = points[points.size()-1];
    //points.push_back(pObject1);
    linspace(points, p1, p2, navigation_msgs::Node::NORTH, 5);
    p2 = points[points.size()-1];
    linspace(points, p2, p3, navigation_msgs::Node::SOUTH, 5);
    p3 = points[points.size()-1];
    //points.push_back(pObject2);

    _test2_previous_node.id_this = -1;
}

void print_graph()
{
    for(int i = 0; i < _graph.num_nodes(); ++i)
    {
        navigation_msgs::Node& node = _graph.get_node(i);
        int north = node.edges[navigation_msgs::Node::NORTH];
        int east = node.edges[navigation_msgs::Node::EAST];
        int south = node.edges[navigation_msgs::Node::SOUTH];
        int west = node.edges[navigation_msgs::Node::WEST];
        int obj = node.edges[navigation_msgs::Node::OBJECT];
        ROS_ERROR("Node %d \t: N=%d E=%d S=%d W=%d Obj=%d, ObjHere=%d",node.id_this,north,east,south,west,obj,node.object_here);
    }
}

void test2_graph(const std::vector<Point>& points)
{
    if (_test2_i >= points.size()) {
        //find shortest path to start node
        navigation_msgs::NextNodeOfInterestRequest request;
        navigation_msgs::NextNodeOfInterestResponse response;

        request.id_from = _test2_previous_node.id_this;
        request.trait = navigation_msgs::NextNodeOfInterestRequest::TRAIT_START;

        print_graph();

        service_next_noi(request, response);
        return;
    }

    navigation_msgs::PlaceNodeRequest request;
    navigation_msgs::PlaceNodeResponse response;

    const Point& p = points[_test2_i];

    request.object_here = false;
    request.east_blocked = true;
    request.west_blocked = true;
    request.north_blocked = true;
    request.south_blocked = true;
    request.id_previous =  _test2_previous_node.id_this;

    if (points[_test2_i].dir == navigation_msgs::Node::OBJECT)
    {
        request.direction = points[_test2_i-1].dir;

        request.object_here = true;
        request.object_type = 0;
        request.object_x = p.x;
        request.object_y = p.y;
    }
    else
    {
        request.direction = p.dir;
        _position.x = p.x;
        _position.y = p.y;
    }

    service_place_node(request, response);

    print_graph();

    _test2_previous_node = response.generated_node;


    _test2_i++;
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
    node = _graph.place_node(3,1.0,place);
    _graph.place_object(node.id_this,place);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph");

    _path.next = 0;

    ros::NodeHandle n;

    _pub_on_node = n.advertise<navigation_msgs::Node>("/navigation/graph/on_node",10);

    ros::Subscriber sub_odom = n.subscribe("/pose/odometry",10,callback_odometry);

    ros::ServiceServer srv_place_ndoe = n.advertiseService("/navigation/graph/place_node",service_place_node);
    ros::ServiceServer srv_next_noi = n.advertiseService("/navigation/graph/next_node_of_interest",service_next_noi);

    navigation_msgs::Node node;

    _graph_viz = boost::shared_ptr<GraphViz>(new GraphViz(_graph, n));

    ros::Rate rate(10.0);
   ///////test
    std::vector<Point> test_points;
    test2_graph_build(test_points);
    std::vector<int> best_path;


    while(n.ok())
    {
        test2_graph(test_points);

        float x = _position.x;
        float y = _position.y;

        if (_graph.on_node(x,y, node)) {
            _pub_on_node.publish(node);
            _graph_viz->highlight_node(node.id_this,true);
        }

        _graph_viz->draw();

        if (_test2_i >= test_points.size()) {
            best_path=find_shortest_path();
             for (int i=0; i<best_path.size();++i){
               std::cout<<best_path[i]<< "->"<<std::endl;
             }

        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
