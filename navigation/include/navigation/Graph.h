#ifndef NAVIGATION_GRAPH_H
#define NAVIGATION_GRAPH_H

#include <navigation_msgs/Node.h>
#include <navigation_msgs/PlaceNodeRequest.h>
#include <common/parameter.h>
#include <common/robot.h>
#include <queue>

#define NAV_GRAPH_UNKNOWN -1
#define NAV_GRAPH_BLOCKED -2

class Graph {
public:

    enum Directions {
        North = 0,
        East,
        South,
        West
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
    void path_to_node(int id_from, int id_to, std::vector<int>& path);

    int get_closest_node(float x, float y, bool consider_obj, double& min_dist);

    bool has_unkown_directions(int id);
    bool is_connected(int id, int id_next);
    bool is_free_connection(int id, int dir);
    bool is_connectable(int id, int dir, int id_next);

    int num_nodes() {return _nodes.size();}

    double get_dist_thresh() {return _dist_thresh();}
    double get_merge_thresh() {return _merge_thresh();}

protected:

    bool on_node(float x, float y, float max_dist, navigation_msgs::Node &node);
    bool on_node_auto_recover(float x, float y, navigation_msgs::PlaceNodeRequest& request, navigation_msgs::Node& node);

    void init_node(navigation_msgs::Node &node,
                   bool blocked_north, bool blocked_east,
                   bool blocked_south, bool blocked_west);
    void set_connected(int id, int dir, int next);

    void update_position(float& x, float& y, float new_x, float new_y);

    void path_to_poi(int id_from, const std::vector<bool>& filter, std::vector<int>& path);

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
    ,_dist_thresh("/navigation/graph/dist_thresh",robot::dim::wheel_distance)
    ,_update_positions("/navigation/graph/update_positions",false)
{
}

void Graph::init_node(navigation_msgs::Node &node,
                      bool blocked_north, bool blocked_east,
                      bool blocked_south, bool blocked_west)
{
    node.id_east = blocked_east ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.id_north = blocked_north ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.id_south = blocked_south ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.id_west = blocked_west ? NAV_GRAPH_BLOCKED : NAV_GRAPH_UNKNOWN;
    node.object_here = false;
}

bool Graph::is_free_connection(int id, int dir)
{
    switch(dir) {
    case navigation_msgs::PlaceNodeRequest::NORTH:
    {
        if (_nodes[id].id_north >= 0) return false;
        break;
    }
    case navigation_msgs::PlaceNodeRequest::EAST:
    {
        if (_nodes[id].id_east >= 0) return false;
        break;
    }
    case navigation_msgs::PlaceNodeRequest::SOUTH:
    {
        if (_nodes[id].id_south >= 0) return false;
        break;
    }
    case navigation_msgs::PlaceNodeRequest::WEST:
    {
        if (_nodes[id].id_west >= 0) return false;
        break;
    }
    default:
    {
        ROS_ERROR("[Graph::is_connectable] Direction %d does not exist",dir);
        return false;
    }
    }
    return true;
}

bool Graph::is_connected(int id, int id_next)
{
    navigation_msgs::Node& node = _nodes[id];

    return
        node.id_north == id_next ||
        node.id_east == id_next  ||
        node.id_south == id_next ||
        node.id_west == id_next;
}

bool Graph::is_connectable(int id, int dir, int id_next)
{
    if (id == -1 || id_next == -1 || id == id_next || is_connected(id, id_next))
    {
        if (is_connected(id, id_next))
            ROS_WARN("Connection %d --> %d already exists.",id, id_next);
        return false;
    }

    navigation_msgs::Node& node = _nodes[id];
    navigation_msgs::Node& next = _nodes[id_next];

    switch(dir) {
    case navigation_msgs::PlaceNodeRequest::NORTH:
    {
        if (node.id_north >= 0) {
            ROS_WARN("Node %d has already a connection in direction N", node.id_this);
            return false;
        }
        if (next.id_south >= 0) {
            ROS_WARN("Node %d has already a connection in direction S", next.id_this);
            return false;
        }
        break;
    }
    case navigation_msgs::PlaceNodeRequest::EAST:
    {
        if (node.id_east >= 0) {
            ROS_WARN("Node %d has already a connection in direction E", node.id_this);
            return false;
        }
        if (next.id_west >= 0) {
            ROS_WARN("Node %d has already a connection in direction W", next.id_this);
            return false;
        }
        break;
    }
    case navigation_msgs::PlaceNodeRequest::SOUTH:
    {
        if (node.id_south >= 0) {
            ROS_WARN("Node %d has already a connection in direction S", node.id_this);
            return false;
        }
        if (next.id_north >= 0) {
            ROS_WARN("Node %d has already a connection in direction N", next.id_this);
            return false;
        }
        break;
    }
    case navigation_msgs::PlaceNodeRequest::WEST:
    {
        if (node.id_west >= 0) {
            ROS_WARN("Node %d has already a connection in direction W", node.id_this);
            return false;
        }
        if (next.id_east >= 0) {
            ROS_WARN("Node %d has already a connection in direction E", next.id_this);
            return false;
        }
        break;
    }
    default:
    {
        ROS_ERROR("[Graph::is_connectable] Direction %d does not exist",dir);
        break;
    }
    }
    return true;
}

void Graph::set_connected(int id, int dir, int id_next)
{
    if (!is_connectable(id, dir, id_next))
        return;

    navigation_msgs::Node& node = _nodes[id];
    navigation_msgs::Node& next = _nodes[id_next];

    switch(dir) {
    case navigation_msgs::PlaceNodeRequest::NORTH:
    {
        node.id_north = id_next;
        next.id_south = id;
        break;
    }
    case navigation_msgs::PlaceNodeRequest::EAST:
    {
        node.id_east = id_next;
        next.id_west = id;
        break;
    }
    case navigation_msgs::PlaceNodeRequest::SOUTH:
    {
        node.id_south = id_next;
        next.id_north = id;
        break;
    }
    case navigation_msgs::PlaceNodeRequest::WEST:
    {
        node.id_west = id_next;
        next.id_east = id;
        break;
    }
    default:
    {
        ROS_ERROR("[Graph::set_connected] Direction %d does not exist",dir);
        break;
    }
    }
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

    while(!is_free_connection(node.id_this,dir)) {
        switch(dir) {
        case navigation_msgs::PlaceNodeRequest::NORTH:
        {
            node = _nodes[node.id_north];
            break;
        }
        case navigation_msgs::PlaceNodeRequest::EAST:
        {
            node = _nodes[node.id_east];
            break;
        }
        case navigation_msgs::PlaceNodeRequest::SOUTH:
        {
            node = _nodes[node.id_south];
            break;
        }
        case navigation_msgs::PlaceNodeRequest::WEST:
        {
            node = _nodes[node.id_west];
            break;
        }
        }
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

    set_connected(id_origin, request.object_direction, node.id_this);

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
    return on_node(x,y, _dist_thresh(), node);
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
    return (n.id_north == NAV_GRAPH_UNKNOWN ||
            n.id_east == NAV_GRAPH_UNKNOWN  ||
            n.id_south == NAV_GRAPH_UNKNOWN ||
            n.id_west == NAV_GRAPH_UNKNOWN);
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

    path_to_poi(id_from, has_unkown, path);
}

void Graph::path_to_next_object(int id_from, std::vector<int> &path)
{
    std::vector<bool> is_object;
    is_object.resize(_nodes.size());
    int i = 0;
    for (std::vector<navigation_msgs::Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it, ++i) {
        is_object[i] = it->object_here;
    }

    path_to_poi(id_from, is_object, path);
}

void Graph::path_to_node(int id_from, int id_to, std::vector<int> &path)
{
    std::vector<bool> is_target_node;
    is_target_node.resize(_nodes.size());
    is_target_node[id_to] = true;

    path_to_poi(id_from, is_target_node, path);
}


void Graph::path_to_poi(int id_from, const std::vector<bool> &filter, std::vector<int> &path)
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
        if (node.id_north >= 0) {
            if(update_dijkstra(id, node.id_north, _nodes, previous, distances))
                queue.push(node.id_north);
        }
        if (node.id_east >= 0) {
            if(update_dijkstra(id, node.id_east, _nodes, previous, distances))
                queue.push(node.id_east);
        }
        if (node.id_south >= 0) {
            if(update_dijkstra(id, node.id_south, _nodes, previous, distances))
                queue.push(node.id_south);
        }
        if (node.id_west >= 0) {
            if(update_dijkstra(id, node.id_west, _nodes, previous, distances))
                queue.push(node.id_west);
        }
    }

    //find closest node where condition is true
    int i_min_dist = 0;
    float min_dist = std::numeric_limits<float>::infinity();

    for (int i = 0; i < _nodes.size(); ++i) {
        if (filter[i] && distances[i] < min_dist) {
            min_dist = distances[i];
            i_min_dist = i;
        }
    }

    //determine shortest path
    int cur = i_min_dist;
    path.push_back(cur);
    while(cur != id_from) {
        cur = previous[cur];
        path.push_back(cur);
    }

    std::reverse(path.begin(), path.end());

}

#endif
