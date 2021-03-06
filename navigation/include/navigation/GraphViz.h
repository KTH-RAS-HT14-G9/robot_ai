#ifndef NAVIGATION_GRAPH_VIZ_H
#define NAVIGATION_GRAPH_VIZ_H

#include "Graph.h"
#include <common/marker_delegate.h>
#include <sstream>

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
            ,id_circle_on(-1)
            ,id_circle_merge(-1)
            ,id_label(-1)
        {
            edges.resize(5,-1);
        }

        int id_node;
        std::vector<int> edges;
        int id_circle_on, id_circle_merge, id_label;
    };

    void draw_node(int id, bool highlight);
    void adjust_data_size();

    Graph& _graph;
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
        //if (_node_clean[i] == false)
        {
            draw_node(i,_highlight.at(i));
            _node_clean.at(i) = true;
            _highlight.at(i) = false;
        }
    }

    _pub_viz.publish(_marker.get());
}



void GraphViz::draw_node(int id, bool highlight)
{
    using namespace navigation_msgs;
    Node& node = _graph.get_node(id);
    MarkerID& marker_id = _marker_ids.at(id);

    static common::Color color_regular(131,178,75);
    static common::Color color_w_obj(222,102,102);

    static common::Color color_regular_h(253,225,41);
    static common::Color color_w_obj_h(102,51,0);

    static common::Color color_edge(200,200,220);
    static common::Color color_unknown(255,148,148);

    static common::Color color_circle_on(148,180,255);
    static common::Color color_circle_merge(242,22,161);
    static common::Color color_object(0,255,0);

    static const float line_z = 0.025f;

    static const float scale = 0.05f;
    static const float thickness = 0.01f;

    common::Color& color_node = highlight ? color_regular_h : color_regular;

//    if (node.object_here) {
//        if (highlight)
//            color_node = color_w_obj_h;
//        else
//            color_node = color_w_obj;
//    }

    //draw node
    if (node.object_here) {
        std::string label = static_cast<std::ostringstream*>( &(std::ostringstream() << node.object_type) )->str();
        marker_id.id_circle_on = _marker.add_circle(node.x,node.y, 0.00001, _graph.get_dist_thresh()*2.0, color_object.r, color_object.g, color_object.b, 50, marker_id.id_circle_on);
        marker_id.id_circle_merge = _marker.add_circle(node.x,node.y, 0.00001, _graph.get_merge_thresh()*2.0, color_circle_merge.r, color_circle_merge.g, color_circle_merge.b, 50, marker_id.id_circle_merge);
        marker_id.id_label = _marker.add_text(node.x,node.y, scale*2.0f, label, 0,255,0, marker_id.id_label);
    }
    else {
        std::string label = static_cast<std::ostringstream*>( &(std::ostringstream() << node.id_this) )->str();
        marker_id.id_node = _marker.add_cube(node.x,node.y,scale, color_node.r, color_node.g, color_node.b, marker_id.id_node);
        marker_id.id_circle_on = _marker.add_circle(node.x,node.y, 0.00001, _graph.get_dist_thresh()*2.0, color_circle_on.r, color_circle_on.g, color_circle_on.b, 50, marker_id.id_circle_on);
        marker_id.id_circle_merge = _marker.add_circle(node.x,node.y, 0.00001, _graph.get_merge_thresh()*2.0, color_circle_merge.r, color_circle_merge.g, color_circle_merge.b, 50, marker_id.id_circle_merge);
        marker_id.id_label = _marker.add_text(node.x,node.y, scale*4.0f, label, 255,255,255, marker_id.id_label);
    }

    for(int i = 0; i < node.edges.size(); ++i)
    {
        if (node.edges[i] >= NAV_GRAPH_UNKNOWN || marker_id.edges[i] != -1) {
            double dx = 0, dy = 0;
            if (i == Node::NORTH) dy = +1.0;
            if (i == Node::EAST)  dx = +1.0;
            if (i == Node::SOUTH) dy = -1.0;
            if (i == Node::WEST)  dx = -1.0;

            if (node.edges[i] == NAV_GRAPH_BLOCKED) {
                dx = 0; dy = 0;
            }

            if (node.edges[i] <= NAV_GRAPH_UNKNOWN)
            {
                double s = 0.2;
                marker_id.edges[i] = _marker.add_line(node.x,node.y, node.x+s*dx, node.y+s*dy, line_z, thickness, color_unknown.r, color_unknown.g, color_unknown.b, marker_id.edges[i]);
            }
            else
            {
                common::Color& color = (i == Node::OBJECT) ? color_object : color_edge;

                double s = scale;
                navigation_msgs::Node& next = _graph.get_node(node.edges[i]);
                marker_id.edges[i] = _marker.add_line(node.x+s*dx,node.y+s*dy, next.x,next.y, line_z, thickness, color.r, color.g, color.b, marker_id.edges[i]);
            }
        }
    }


}

void GraphViz::highlight_node(int id, bool flag)
{
    adjust_data_size();
    if (id >= _graph.num_nodes())
        return;

    _highlight.at(id) = flag;
    _node_clean.at(id) = false;
}

#endif
