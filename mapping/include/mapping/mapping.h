#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ir_converter/Distance.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
using std::vector;

class Mapping
{
public:
    Mapping();
    void distanceCallback(const ir_converter::Distance::ConstPtr&);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr&);
    void updateProbGrid();
    void updateOccupancyGrid();
private:
    void initProbGrid();
    void initOccupancyGrid();
    void updateFLCell();
    ros::NodeHandle handle;
    ros::Subscriber distance_sub;
    ros::Subscriber odometry_sub;

    vector<vector<double> > prob_grid;
    vector<vector<int> > occupancy_grid;
    int min_y, max_y, min_x, max_x;

    //odometry
    ros::Time timestamp;
    double position_x, position_y;
    double orientation_x, orientation_y, orientation_z, orientation_w;

    double fl_side, fr_side, bl_side, br_side, l_front, r_front;

    static const int GRID_HEIGHT;
    static const int GRID_WIDTH;
    static const int GRID_RESOLUTION;
    static const double P_PRIOR;
    static const double P_OCC;
    static const double P_FREE;

    static const int UNKNOWN;
    static const int FREE;
    static const int OCCUPIED;
    static const int BLUE_CUBE;
    static const int RED_SPHERE;
};

#endif // MAPPING_H
