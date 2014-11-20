#include "mapping/mapping.h"

const int Mapping::GRID_HEIGHT = 1000;
const int Mapping::GRID_WIDTH = 1000;
const int Mapping::GRID_RESOLUTION = 10000; // 10 000 per sq meter = 1 per sq cm
const double Mapping::P_PRIOR = 0.5;
const double Mapping::P_OCC = 0.7;
const double Mapping::P_FREE  = 0.35;

const int Mapping::UNKNOWN = -1;
const int Mapping::FREE = 0;
const int Mapping::OCCUPIED = 1;
const int Mapping::BLUE_CUBE = 2;
const int Mapping::RED_SPHERE = 3; // etc

Mapping::Mapping()
{
    handle = ros::NodeHandle("");
    distance_sub = handle.subscribe("/perception/ir/distance", 1000, &Mapping::distanceCallback, this);
    odometry_sub = handle.subscribe("/pose/odometry/", 1000, &Mapping::odometryCallback, this);

    initProbGrid();
    initOccupancyGrid();
}

void Mapping::distanceCallback(const ir_converter::Distance::ConstPtr& distance)
{
    fl_side = distance->fl_side;
    bl_side = distance->bl_side;
    fr_side = distance->fr_side;
    br_side = distance->br_side;
    l_front = distance->l_front;
    r_front = distance->r_front;
}

void Mapping::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    position_x = odom->pose.pose.position.x;
    position_y = odom->pose.pose.position.y;
    orientation_x = odom->pose.pose.orientation.x;
    orientation_y = odom->pose.pose.orientation.y;
    orientation_z = odom->pose.pose.orientation.z;
    orientation_w = odom->pose.pose.orientation.w;
}

void Mapping::updateProbGrid()
{
    updateFLCell();
    //FR, BL, BR
}

void Mapping::updateFLCell()
{

}

void Mapping::updateOccupancyGrid()
{

}

void Mapping::initProbGrid()
{
    prob_grid.resize(GRID_HEIGHT);
    for(int i = 0; i < GRID_HEIGHT; ++i)
        prob_grid[i].resize(GRID_WIDTH);

    for(int i = 0; i < GRID_HEIGHT; ++i)
        for(int j = 0; j < GRID_WIDTH; ++j)
            prob_grid[i][j] = P_PRIOR;
}

void Mapping::initOccupancyGrid()
{
    prob_grid.resize(GRID_HEIGHT);
    for(int i = 0; i < GRID_HEIGHT; ++i)
        prob_grid[i].resize(GRID_WIDTH);

    for(int i = 0; i < GRID_HEIGHT; ++i)
        for(int j = 0; j < GRID_WIDTH; ++j)
            prob_grid[i][j] = UNKNOWN;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping");
    Mapping mapping;
    ros::Rate loop_rate(10); // what should this be?

    while(ros::ok())
    {
        // do stuff
        ros::spinOnce();
        loop_rate.sleep();
    }
}
