#ifndef NAVIGATION_GRAPH_H
#define NAVIGATION_GRAPH_H

#include <navigation_msgs/Node.h>
#include <navigation_msgs/PlaceNodeRequest.h>
#include <navigation_msgs/Graph.h>
#include <common/parameter.h>
#include <common/robot.h>
#include <queue>
#include <ros/serialization.h>
#include <fstream>

#define NAV_GRAPH_UNKNOWN -1
#define NAV_GRAPH_BLOCKED -2

const char* DirectionNames[] = {"North","East","South","West","Object"};

class Graph {
public:

    enum Directions {
        North = 0,
        East,
        South,
        West,
        Object
    };

    Graph();

    navigation_msgs::Node& place_node(float x, float y,
                                      navigation_msgs::PlaceNodeRequest& request);
    navigation_msgs::Node& place_object(int id_origin,
                                        navigation_msgs::PlaceNodeRequest& request);

    bool on_node(float x, float y, navigation_msgs::Node &node);
    bool on_object_node(float x, float y, navigation_msgs::Node& node);

    navigation_msgs::Node& get_node(int id);

    void path_to_next_unknown(int id_from, std::vector<int>& path);
    void path_to_next_object(int id_from, std::vector<int>& path);
    void path_to_node(int id_from, int id_to, std::vector<int>& path, double& dist);

    int get_closest_node(float x, float y, bool consider_obj, double& min_dist);

    bool has_unkown_directions(int id);
    bool is_connected(int id, int id_next);
    bool is_free_connection(int id, int dir);
    bool is_connectable(int id, int dir, int id_next);

    int num_nodes() {return _nodes.size();}

    double get_dist_thresh() {return _dist_thresh();}
    double get_merge_thresh() {return _merge_thresh();}

    void read_from_msg(const navigation_msgs::GraphConstPtr& msg);
    void publish_to_topic(ros::Publisher& pub);

protected:

    bool on_node(float x, float y, float max_dist, navigation_msgs::Node &node);
    bool on_node_auto_recover(float x, float y, navigation_msgs::PlaceNodeRequest& request, navigation_msgs::Node& node);

    void init_node(navigation_msgs::Node &node,
                   bool blocked_north, bool blocked_east,
                   bool blocked_south, bool blocked_west);
    void set_connected(int id, int dir, int next);

    void update_blocked_edges(navigation_msgs::Node& node, navigation_msgs::PlaceNodeRequest& request);
    void update_position(float& x, float& y, float new_x, float new_y);

    void path_to_poi(int id_from, const std::vector<bool>& filter, std::vector<int>& path, double& dist);

    inline int invert_direction(int dir) {
        if (dir == Object) return dir;
        dir = (dir+2)%4;
    }

    int panic_forwarding(int id, int dir);

    std::vector<navigation_msgs::Node> _nodes;
    int _next_node_id;

    Parameter<double> _dist_thresh;
    Parameter<double> _merge_thresh;
    Parameter<bool> _update_positions;
};

Graph::Graph()
    :_next_node_id(0)
    ,_merge_thresh("/navigation/graph/merge_thresh",robot::dim::wheel_distance/1.5)
    ,_dist_thresh("/navigation/graph/dist_thresh",robot::dim::wheel_distance*0.9)
    ,_update_positions("/navigation/graph/update_positions",false)
{
}

void Graph::init_node(navigation_msgs::Node &node,
                      bool blocked_north, bool blocked_east,
                      bool blocked_south, bool blocked_west)
{
    node.edges.resize(5);
    node.edges[North] = blocked_north ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.edges[East] = blocked_east ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.edges[South] = blocked_south ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.edges[West] = blocked_west ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.edges[Object] = NAV_GRAPH_BLOCKED;

    node.object_here = false;
}

bool Graph::is_free_connection(int id, int dir)
{
    return _nodes[id].edges[dir] < 0;
}

bool Graph::is_connected(int id, int id_next)
{
    navigation_msgs::Node& node = _nodes[id];

    for(int i = 0; i < node.edges.size(); ++i)
        if (node.edges[i] == id_next)
            return true;

    return false;
}

bool Graph::is_connectable(int id, int dir, int id_next)
{
    if (id < 0 || id_next < 0 || id == id_next)
        return false;

    navigation_msgs::Node& node = _nodes[id];
    navigation_msgs::Node& next = _nodes[id_next];

    if (node.edges[dir] >= 0) {
        //ROS_WARN("Node %d has already a connection in direction %s", node.id_this, DirectionNames[dir]);
        return false;
    }
    if (next.edges[invert_direction(dir)] >= 0) {
        //ROS_WARN("Node %d has already a connection in direction %s", next.id_this, DirectionNames[dir]);
        return false;
    }

    return true;
}

void Graph::set_connected(int id, int dir, int id_next)
{
    if (!is_connectable(id, dir, id_next))
        return;

    navigation_msgs::Node& node = _nodes[id];
    navigation_msgs::Node& next = _nodes[id_next];

    node.edges[dir] = id_next;
    next.edges[invert_direction(dir)] = id;
}

void Graph::update_position(float& x, float& y, float new_x, float new_y)
{
    if (_update_positions()) {
        x = 0.3*x + 0.7*new_x;
        y = 0.3*y + 0.7*new_y;
    }
}

/**
  * Panic forwarding for trying to sustain a reasonable graph.
  * Starting at the node with id,
  * moves in the given direction, until a node with a free edge
  * in direction dir is availbale.
  */
int Graph::panic_forwarding(int id, int dir)
{
    navigation_msgs::Node& node = _nodes[id];

    while(!is_free_connection(node.id_this,dir))
    {
        node = _nodes[node.edges[dir]];
    }

    return node.id_this;
}

bool Graph::on_node_auto_recover(float x, float y, navigation_msgs::PlaceNodeRequest& request, navigation_msgs::Node& node)
{
    if (request.id_previous == -1) return false;

    if (!on_node(x,y, _merge_thresh(), node)) {
        //check if there is a free edge at the previous id
        if (is_free_connection(request.id_previous, request.direction))
            return false;
        else
        {
            //determine closest
            double dist;
            int closest = get_closest_node(x,y, false, dist);

            request.id_previous = closest;

            node = _nodes[closest];

            return true;
        }
    }
    else return true;
}

void Graph::update_blocked_edges(navigation_msgs::Node& node, navigation_msgs::PlaceNodeRequest& request)
{
    if (node.edges[North] <= NAV_GRAPH_UNKNOWN) node.edges[North] = request.east_blocked;
    if (node.edges[East] <= NAV_GRAPH_UNKNOWN) node.edges[East] = request.east_blocked;
    if (node.edges[South] <= NAV_GRAPH_UNKNOWN) node.edges[South] = request.east_blocked;
    if (node.edges[West] <= NAV_GRAPH_UNKNOWN) node.edges[West] = request.east_blocked;
}

navigation_msgs::Node& Graph::place_node(float x, float y, navigation_msgs::PlaceNodeRequest &request)
{
    navigation_msgs::Node node;

    //only place node, if there is no other node close nearby
    if (!on_node_auto_recover(x,y,request,node)) {

        init_node(node, request.north_blocked, request.east_blocked, request.south_blocked, request.west_blocked);

        node.x = x;
        node.y = y;

        node.id_this = _nodes.size();

        _nodes.push_back(node);
    }
    else {
        ROS_INFO("On node %d", node.id_this);
        update_blocked_edges(node, request);
        update_position(_nodes[node.id_this].x, _nodes[node.id_this].y, x, y);
    }

    if (is_connectable(request.id_previous, request.direction, node.id_this))
        set_connected(request.id_previous, request.direction, node.id_this);

    return _nodes[node.id_this];
}

navigation_msgs::Node& Graph::place_object(int id_origin, navigation_msgs::PlaceNodeRequest &request)
{
    navigation_msgs::Node neighbor;
    bool has_neighbor = on_object_node(request.object_x,request.object_y, neighbor);

    navigation_msgs::Node node;

    //only place object node, if there is no other object node close nearby,
    if (!has_neighbor) {

        init_node(node, true, true, true, true);

        node.object_here = true;
        node.x = request.object_x;
        node.y = request.object_y;

        node.id_this = _nodes.size();

        _nodes.push_back(node);
    }
    else {
        node = neighbor;
        update_position(_nodes[node.id_this].x, _nodes[node.id_this].y, request.object_x, request.object_y);
    }

    set_connected(id_origin, Object, node.id_this);

    return _nodes[node.id_this];
}

navigation_msgs::Node& Graph::get_node(int id)
{
    if (id < 0 || id >= _nodes.size())
        ROS_ERROR("[Graph::get_node] id: %d out of array bounds",id);

    return _nodes.at(id);
}

double sq_dist(double x0, double y0, double x1, double y1)
{
    return (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0);
}

bool Graph::on_node(float x, float y, navigation_msgs::Node &node)
{
    double dist;
    int i = get_closest_node(x,y, true, dist);

    if (dist <= _dist_thresh())
    {
        node = _nodes[i];
        return true;
    }
    return false;
}

int Graph::get_closest_node(float x, float y, bool consider_obj, double& min_dist)
{
    int i = 0;
    int closest = -1;
    min_dist = std::numeric_limits<double>::infinity();

    for(std::vector<navigation_msgs::Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it, ++i)
    {
        if (consider_obj == it->object_here) {

            double sq_d = sq_dist(x,y, it->x, it->y);

            if (sq_d < min_dist) {
                min_dist = sq_d;
                closest = i;
            }
        }
    }

    return closest;
}

bool Graph::on_object_node(float x, float y, navigation_msgs::Node& node)
{
    double sq_dist_thresh = _merge_thresh();
    sq_dist_thresh *= sq_dist_thresh;

    double min_dist;
    int closest = get_closest_node(x,y,true,min_dist);
    if (closest == -1) return false;

    if (min_dist < sq_dist_thresh) {
        node = _nodes[closest];
        return true;
    }

    return false;
}

bool Graph::on_node(float x, float y, float max_dist, navigation_msgs::Node &node)
{
    double sq_dist_thresh = max_dist;
    sq_dist_thresh *= sq_dist_thresh;

    double min_dist;
    int closest = get_closest_node(x,y,false,min_dist);
    if (closest == -1) return false;

    if (min_dist < sq_dist_thresh) {
        node = _nodes[closest];
        return true;
    }

    return false;
}

bool Graph::has_unkown_directions(int id)
{
    navigation_msgs::Node& n = _nodes[id];
    for (int i = 0; i < 4; ++i) //only consider N,E,S,W
    {
        if (n.edges[i] == NAV_GRAPH_UNKNOWN)
            return true;
    }
    return false;
}

bool update_dijkstra(int id,
                     int id_next,
                     std::vector<navigation_msgs::Node>& nodes,
                     std::vector<int>& previous,
                     std::vector<float>& distances)
{
    navigation_msgs::Node& node = nodes[id];
    navigation_msgs::Node& next = nodes[id_next];
    float d = distances[id] + sq_dist(node.x,node.y, next.x, next.y);

    if (d < distances[id_next]) {
        distances[id_next] = d;
        previous[id_next] = id;
        return true;
    }
    return false;
}

void Graph::path_to_next_unknown(int id_from, std::vector<int>& path)
{
    std::vector<bool> has_unkown;
    has_unkown.resize(_nodes.size());
    int i = 0;
    for (std::vector<navigation_msgs::Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it, ++i) {
        has_unkown[i] = has_unkown_directions(i);
    }

    double dummy;
    path_to_poi(id_from, has_unkown, path, dummy);
}

void Graph::path_to_next_object(int id_from, std::vector<int> &path)
{
    std::vector<bool> is_object;
    is_object.resize(_nodes.size());
    int i = 0;
    for (std::vector<navigation_msgs::Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it, ++i) {
        is_object[i] = it->object_here;
    }

    double dummy;
    path_to_poi(id_from, is_object, path, dummy);
}

void Graph::path_to_node(int id_from, int id_to, std::vector<int> &path, double& dist)
{
    std::vector<bool> is_target_node;
    is_target_node.resize(_nodes.size());
    is_target_node[id_to] = true;

//    std::cout << "From " << id_from << " to " << id_to << " Filter: ";
//    for(int i = 0; i < is_target_node.size(); ++i)
//        std::cout << "i = " << i << ": " << is_target_node[i] << ", ";
//    std::cout << std::endl;

    path_to_poi(id_from, is_target_node, path, dist);
}


void Graph::path_to_poi(int id_from, const std::vector<bool> &filter, std::vector<int> &path, double &dist)
{
    if (_nodes.size() == 0)
        return;

    path.clear();
    std::queue<int> queue;

    std::vector<int> previous;
    std::vector<float> distances;
    std::vector<bool> visited;

    previous.resize(_nodes.size(),-1);
    distances.resize(_nodes.size(), std::numeric_limits<float>::infinity());
    visited.resize(_nodes.size(),false);

    distances[id_from] = 0;

    queue.push(id_from);

    while(!queue.empty()) {
        int id = queue.front();
        queue.pop();

        //check if node already visited
        if (visited[id] == true)
            continue;

        visited[id] = true;

        navigation_msgs::Node& node = _nodes[id];
        for(int i = 0; i < node.edges.size(); ++i)
        {
            if (i==Object) {
                int a = 5;
                a*=3;
            }
            if (node.edges[i] >= 0) {
                if(update_dijkstra(id, node.edges[i], _nodes, previous, distances))
                    queue.push(node.edges[i]);
            }
        }

    }

    //find closest node where condition is true
    int i_min_dist = -1;
    float min_dist = std::numeric_limits<float>::infinity();

    for (int i = 0; i < _nodes.size(); ++i) {
//        std::cout << i << ": " << filter[i] << ", dist: " << distances[i] << std::endl;
        if (filter[i] && distances[i] < min_dist) {
            min_dist = distances[i];
            i_min_dist = i;
        }
    }

    //no node to reach
    if (i_min_dist == -1) {
        path.push_back(id_from);
        return;
    }

    dist = min_dist;

    //determine shortest path
    int cur = i_min_dist;
    path.push_back(cur);
    while(cur != id_from) {
        cur = previous[cur];
        path.push_back(cur);
    }

    std::reverse(path.begin(), path.end());

}

void Graph::publish_to_topic(ros::Publisher& pub)
{
    navigation_msgs::Graph graph;
    graph.nodes.reserve(_nodes.size());
    for(int i = 0; i < _nodes.size(); ++i) {
        graph.nodes.push_back(_nodes[i]);
    }
    pub.publish(graph);
}

void Graph::read_from_msg(const navigation_msgs::GraphConstPtr& msg)
{
    _nodes.clear();
    _nodes.reserve(msg->nodes.size());
    _next_node_id = msg->nodes.size();

    for(int i = 0; i < msg->nodes.size(); ++i)
    {
        _nodes.push_back(msg->nodes[i]);
    }
}

//void Graph::save_to_file()
//{
//    ROS_ERROR("Saving...");

//    std::ofstream out("graph.txt");
//    std::ofstream out_size("graph_size.txt");

//    uint32_t serial_size = ros::serialization::serializationLength(_nodes);

//    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

//    ros::serialization::OStream stream(buffer.get(), serial_size);
//    ros::serialization::serialize(stream, _nodes);

//    uint8_t* data = stream.getData();
//    out.write((char*)data, stream.getLength());
//    out.close();

//    out_size << serial_size << "\n" << _nodes.size();
//    out_size.close();
//}

//void Graph::load_from_file()
//{
//    ROS_ERROR("Loading...");

//    uint32_t serial_size;
//    uint32_t num_nodes;

//    //load number of nodes
//    std::ifstream in_size("graph_size.txt");

//    std::string line;
//    std::getline(in_size, line);
//    std::stringstream ss(line);
//    ss >> serial_size;

//    line.clear();
//    std::getline(in_size, line);
//    std::stringstream ss2(line);
//    ss2 >> num_nodes;

//    ROS_ERROR("Recovered size: %d, nodes: %d",serial_size, num_nodes);

//    std::ifstream in("graph.txt");

//    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
//    uint8_t* buf = buffer.get();
//    in.read((char*)buf, serial_size);

//    std::vector<navigation_msgs::Node> nodes;
//    nodes.resize(num_nodes);

//    // Fill buffer with serialized nodes
//    ros::serialization::IStream stream(buffer.get(), serial_size);
//    ros::serialization::deserialize(stream, nodes);

//    ROS_ERROR("Recovered nodes: %ld", nodes.size());
//}

#endif
