#ifndef NAVIGATION_GRAPH_VIZ_H
#define NAVIGATION_GRAPH_VIZ_H

#include "Graph.h"
#include <common/marker_delegate.h>

class GraphViz {
public:

    GraphViz(Graph& graph, ros::NodeHandle& n);

    void draw();
    void highlight_node(int id, bool flag);

protected:

    class MarkerID {
    public:
        MarkerID()
            :id_node(-1)
            ,id_north(-1)
            ,id_east(-1)
            ,id_south(-1)
            ,id_west(-1) {}
        MarkerID(int id, int north, int east, int south, int west)
            :id_node(id)
            ,id_north(north)
            ,id_east(east)
            ,id_south(south)
            ,id_west(west) {}

        int id_node;
        int id_north, id_east, id_south, id_west;
    };

    void draw_node(int id, bool highlight);
    void adjust_data_size();

    Graph _graph;
    std::vector<bool> _node_clean;
    std::vector<bool> _highlight;
    std::vector<MarkerID> _marker_ids;

    ros::Publisher _pub_viz;
    common::MarkerDelegate _marker;
};

GraphViz::GraphViz(Graph &graph, ros::NodeHandle &n)
    :_graph(graph)
    ,_marker("map","topo_graph")
{
    _pub_viz = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);


//    for(int i = 0; i < 100; ++i) {
//        _marker.add_cube(i,i,1,0,0,0,-1);
//        //_marker.add_line(i,0,0,i,0,0.1,0,0,0,-1);
//    }
}

void GraphViz::adjust_data_size()
{
    int N = _graph.num_nodes();
    int K = _marker_ids.size();
    for(int i = 0; i < N-K; ++i) {
        _node_clean.push_back(false);
        _marker_ids.push_back(MarkerID());
        _highlight.push_back(false);
    }
}

void GraphViz::draw()
{
    int N = _graph.num_nodes();
    adjust_data_size();

    for(int i = 0; i < N; ++i)
    {
        if (_node_clean[i] == false) {
            draw_node(i,_highlight.at(i));
            _node_clean.at(i) = true;
            _highlight.at(i) = false;
        }
    }

    _pub_viz.publish(_marker.get());
}



void GraphViz::draw_node(int id, bool highlight)
{
    navigation_msgs::Node& node = _graph.get_node(id);
    MarkerID& marker_id = _marker_ids.at(id);

    static common::Color color_regular(131,178,75);
    static common::Color color_w_obj(222,102,102);

    static common::Color color_regular_h(253,225,41);
    static common::Color color_w_obj_h(102,51,0);

    static common::Color color_edge(200,200,220);
    static common::Color color_unknown(100,100,100);

    static const float line_z = 0.025f;

    static const float scale = 0.05f;
    static const float thickness = 0.01f;

    common::Color& color_node = highlight ? color_regular_h : color_regular;

    if (node.object_here) {
        if (highlight)
            color_node = color_w_obj_h;
        else
            color_node = color_w_obj;
    }

    //draw node
    std::string label = ""+node.id_this;
    marker_id.id_node = _marker.add_cube(node.x,node.y,scale, color_node.r, color_node.g, color_node.b, label, marker_id.id_node);

    //draw edges
    if (node.id_north >= NAV_GRAPH_UNKNOWN) {
        if (node.id_north == NAV_GRAPH_UNKNOWN)
            marker_id.id_north = _marker.add_line(node.x,node.y, node.x, node.y+0.2, line_z, thickness, color_unknown.r, color_unknown.g, color_unknown.b, marker_id.id_north);
        else {
            navigation_msgs::Node& next = _graph.get_node(node.id_north);
            marker_id.id_north = _marker.add_line(node.x,node.y, next.x,next.y, line_z, thickness, color_edge.r, color_edge.g, color_edge.b, marker_id.id_north);
        }
    }

    if (node.id_east >= NAV_GRAPH_UNKNOWN) {
        if (node.id_east == NAV_GRAPH_UNKNOWN)
            marker_id.id_east = _marker.add_line(node.x,node.y, node.x+0.2, node.y, line_z, thickness, color_unknown.r, color_unknown.g, color_unknown.b, marker_id.id_east);
        else {
            navigation_msgs::Node& next = _graph.get_node(node.id_east);
            marker_id.id_east = _marker.add_line(node.x,node.y, next.x,next.y, line_z, thickness, color_edge.r, color_edge.g, color_edge.b, marker_id.id_east);
        }
    }

    if (node.id_south >= NAV_GRAPH_UNKNOWN) {
        if (node.id_south == NAV_GRAPH_UNKNOWN)
            marker_id.id_south = _marker.add_line(node.x,node.y, node.x, node.y-0.2, line_z, thickness, color_unknown.r, color_unknown.g, color_unknown.b, marker_id.id_south);
        else {
            navigation_msgs::Node& next = _graph.get_node(node.id_south);
             marker_id.id_south = _marker.add_line(node.x,node.y, next.x,next.y, line_z, thickness, color_edge.r, color_edge.g, color_edge.b, marker_id.id_south);
        }
    }

    if (node.id_west >= NAV_GRAPH_UNKNOWN) {
        if (node.id_west == NAV_GRAPH_UNKNOWN)
            marker_id.id_west = _marker.add_line(node.x,node.y, node.x-0.2, node.y, line_z, thickness, color_unknown.r, color_unknown.g, color_unknown.b, marker_id.id_west);
        else {
            navigation_msgs::Node& next = _graph.get_node(node.id_west);
            marker_id.id_west = _marker.add_line(node.x,node.y, next.x,next.y, line_z, thickness, color_edge.r, color_edge.g, color_edge.b, marker_id.id_west);
        }
    }


}

void GraphViz::highlight_node(int id, bool flag)
{
    adjust_data_size();
    _highlight.at(id) = flag;
    _node_clean.at(id) = false;
}

#endif
