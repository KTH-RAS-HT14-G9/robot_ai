#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ir_converter/Distance.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

using std::vector;

template<class T>
struct Point
{
    T x;
    T y;

    Point() {}
    Point(T x, T y) : x(x), y(y) {}
};

class Mapping
{
public:
    Mapping();
    void distanceCallback(const ir_converter::Distance::ConstPtr&);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr&);
    void startTurnCallback(const std_msgs::Float64::ConstPtr&);
    void stopTurnCallback(const std_msgs::Bool::ConstPtr&);
    void updateGrid();
    void publishMap();
    void broadcastTransform();
    void waitForTransform();

private:
    void markPointsFreeBetween(Point<double> p1, Point<double> p2);
    void updateIR(Point<double> ir, Point<double> obstacle);
    void markPointOccupied(Point<double> p);
    void markPointFree(Point<double> p);
    Point<int> robotPointToCell(Point<double> p);
    Point<double> robotToMapTransform(Point<double> p);
    Point<int> mapPointToCell(Point<double> p);
    void markProbabilityGrid(Point<int> cell, double log_prob);
    void updateOccupancyGrid(Point<int>);
    bool isIRValid(double reading);
    void initProbabilityGrid();
    void initOccupancyGrid();

    ros::NodeHandle handle;
    ros::Subscriber distance_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber start_turn_sub;
    ros::Subscriber stop_turn_sub;
    ros::Publisher pc_pub;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    vector<vector<double> > prob_grid;
    vector<vector<int> > occ_grid;
    bool turning;
    Point<double> pos;
    double fl_ir, fr_ir, bl_ir, br_ir;

    static const double INVALID_READING;
    static const double MAP_HEIGHT, MAP_WIDTH;
    static const int GRID_HEIGHT, GRID_WIDTH;
    static const double MAP_X_OFFSET, MAP_Y_OFFSET;
    static const double P_PRIOR, P_OCC, P_FREE;
    static const double FREE_OCCUPIED_THRESHOLD;

    static const int UNKNOWN, FREE, OCCUPIED;
    static const int BLUE_CUBE;
    static const int RED_SPHERE;
  };

#endif // MAPPING_H
