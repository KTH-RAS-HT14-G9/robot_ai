#include "mapping/mapping.h"

const int Mapping::GRID_HEIGHT = 1000;
const int Mapping::GRID_WIDTH = 1000;

const double Mapping::MAP_HEIGHT = 10.0;
const double Mapping::MAP_WIDTH = 10.0;
const double Mapping::MAP_X_OFFSET = MAP_WIDTH/2.0;
const double Mapping::MAP_Y_OFFSET = MAP_HEIGHT/2.0;

const double Mapping::P_PRIOR = log(0.5);
const double Mapping::P_OCC = log(0.7);
const double Mapping::P_FREE  = log(0.35);

const double Mapping::FREE_OCCUPIED_THRESHOLD = log(0.5);

const double Mapping::INVALID_READING = -1.0;
const int Mapping::UNKNOWN = -1;
const int Mapping::FREE = 0;
const int Mapping::OCCUPIED = 1;
const int Mapping::BLUE_CUBE = 2;
const int Mapping::RED_SPHERE = 3;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointXYZI PCPoint;

Mapping::Mapping() :
    fl_ir(INVALID_READING), fr_ir(INVALID_READING),
    bl_ir(INVALID_READING), br_ir(INVALID_READING),
    pos(Point<double>(0.0,0.0)), turning(false)
{
    handle = ros::NodeHandle("");
    distance_sub = handle.subscribe("/perception/ir/distance", 1, &Mapping::distanceCallback, this);
    odometry_sub = handle.subscribe("/pose/odometry/", 1, &Mapping::odometryCallback, this);
    start_turn_sub = handle.subscribe("/controller/turn/angle", 1, &Mapping::startTurnCallback, this);
    stop_turn_sub = handle.subscribe("/controller/turn/done", 1, &Mapping::stopTurnCallback, this);
    pc_pub = handle.advertise<PointCloud>("/mapping/point_cloud", 1);

    initProbabilityGrid();
    initOccupancyGrid();
 //   broadcastTransform();
}

void Mapping::updateGrid()
{
    //if(turning)
        //return;

    if(isIRValid(fl_ir))
    {
        double ir_x_offset = robot::ir::offset_front_left_forward;
        Point<double> ir(ir_x_offset, 0);
        Point<double> obstacle(ir_x_offset,fl_ir);
        updateIR(ir, obstacle);
    }

    if(isIRValid(fr_ir))
    {
        double ir_x_offset = robot::ir::offset_front_right_forward;
        Point<double> ir(ir_x_offset, 0);
        Point<double> obstacle(ir_x_offset,-fr_ir);
        updateIR(ir, obstacle);
    }

    if(isIRValid(bl_ir))
    {
        double ir_x_offset = -1.0*robot::ir::offset_rear_left_forward;
        Point<double> ir(ir_x_offset, 0);
        Point<double> obstacle(ir_x_offset, bl_ir);
        updateIR(ir, obstacle);
    }

    if(isIRValid(br_ir))
    {
        double ir_x_offset = -1.0*robot::ir::offset_rear_right_forward;
        Point<double> ir(ir_x_offset, 0);
        Point<double> obstacle(ir_x_offset, -br_ir);
        updateIR(ir, obstacle);
    }

    //  TODO update cells robot is at to free
}

void Mapping::markPointsFreeBetween(Point<double> p1, Point<double> p2)
{
    Point<double> min = p1.x <= p2.x ? p1 : p2;
    Point<double> max = p1.x > p2.x ? p1 : p2;
    double dx = max.x-min.x;
    double dy = max.y-min.y;

    if(dx < 0.01) {
        min = p1.y <= p2.y ? p1 : p2;
        max = p1.y > p2.y ? p1 : p2;
        double x = min.x;
        for(double y = min.y; y < max.y; y +=0.01)
        {
            markPointFree(Point<double>(x,y));
        }
    } else {
        for(double x = min.x; x < max.x; x+=0.01)
        {
            double y = min.y + dy * (x-min.x) / dx;
            markPointFree(Point<double>(x,y));
        }
    }
}

void Mapping::updateIR(Point<double> ir, Point<double> obstacle)
{
    markPointOccupied(obstacle); // only seems to work on the right
    markPointsFreeBetween(ir, obstacle); // ^__ probably overwritten to free here
}

void Mapping::markPointOccupied(Point<double> point)
{
    Point<int> cell = robotPointToCell(point);
    markProbabilityGrid(cell, P_OCC);
    updateOccupancyGrid(cell);
}

void Mapping::markPointFree(Point<double> point)
{
    Point<int> cell = robotPointToCell(point);
    markProbabilityGrid(cell, P_FREE);
    updateOccupancyGrid(cell);
}

Point<int> Mapping::robotPointToCell(Point<double> point)
{
    Point<double> map_point = robotToMapTransform(point);
    return mapPointToCell(map_point);
}

Point<double> Mapping::robotToMapTransform(Point<double> point)
{
    geometry_msgs::PointStamped robot_point;
    robot_point.header.frame_id = "robot";
    robot_point.header.stamp = ros::Time::now();
    robot_point.point.x = point.x;
    robot_point.point.y = point.y;
    robot_point.point.z = 0.0;

    geometry_msgs::PointStamped map_point;
    tf_listener.transformPoint("map", robot_point, map_point);
    Point<double> res = Point<double>(map_point.point.x + MAP_X_OFFSET, map_point.point.y + MAP_Y_OFFSET);
    //ROS_INFO("robot x: %f y: %f, map x: %f, y: %f", point.x, point.y, res.x, res.y);
    return res;
}

Point<int> Mapping::mapPointToCell(Point<double> pos)
{
    int x = round(pos.x*100.0);
    int y = round(pos.y*100.0);
    return Point<int>(x,y);
}

void Mapping::markProbabilityGrid(Point<int> cell, double log_prob)
{
    prob_grid[cell.y][cell.x] += log_prob - P_PRIOR;
}

void Mapping::updateOccupancyGrid(Point<int> cell)
{
    if(prob_grid[cell.y][cell.x] > FREE_OCCUPIED_THRESHOLD)
        occ_grid[cell.y][cell.x] = OCCUPIED;
    else if(prob_grid[cell.y][cell.x] < FREE_OCCUPIED_THRESHOLD)
        occ_grid[cell.y][cell.x] = FREE;
    else
        occ_grid[cell.y][cell.x] = UNKNOWN;
}

bool Mapping::isIRValid(double value)
{
    return value > 0.0 && value < 0.8;
}

void Mapping::initProbabilityGrid()
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
    occ_grid.resize(GRID_HEIGHT);
    for(int i = 0; i < GRID_HEIGHT; ++i)
        occ_grid[i].resize(GRID_WIDTH);

    for(int i = 0; i < GRID_HEIGHT; ++i)
        for(int j = 0; j < GRID_WIDTH; ++j)
            occ_grid[i][j] = UNKNOWN;
}

void Mapping::broadcastTransform()
{
    tf_broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                    ros::Time(),"map", "robot"));
}

void Mapping::distanceCallback(const ir_converter::Distance::ConstPtr& distance)
{   
    fl_ir = distance->fl_side;
    bl_ir = distance->bl_side;
    fr_ir = distance->fr_side;
    br_ir = distance->br_side;
}

void Mapping::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    pos = Point<double>(x,y);
}

void Mapping::startTurnCallback(const std_msgs::Float64::ConstPtr & angle)
{
    turning = true;
}

void Mapping::stopTurnCallback(const std_msgs::Bool::ConstPtr & var)
{
    turning = false;
}

void Mapping::waitForTransform()
{
    tf_listener.waitForTransform("robot", "map",
                                  ros::Time::now(), ros::Duration(3.0));
}

void Mapping::publishMap()
{
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "map";
    msg->height = GRID_HEIGHT;
    msg->width = GRID_WIDTH;

    int numFree = 0;
    int numUnknown = 0;
    int numOcc = 0;

    for(int i = 0; i < GRID_HEIGHT; ++i)
    {
        for(int j = 0; j < GRID_WIDTH; ++j)
        {
            int cell = occ_grid[i][j];

            if(cell == OCCUPIED)
                ++numOcc;
            else if(cell == FREE)
                ++numFree;
            else if(cell == UNKNOWN)
                ++numUnknown;

            double intensity = (cell == FREE) ? 0.0 : (cell == OCCUPIED) ? 1.0 : 0.5;

            PCPoint p(intensity);
            p.x = (double) j/100.0;
            p.y = (double) i/100.0;
            msg->points.push_back(p);
        }
    }
    ROS_INFO("F: %d, O: %d, U: %d", numFree, numOcc, numUnknown);

    pc_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping");
    Mapping mapping;
    ros::Rate loop_rate(10); // what should this be?
    int counter = 0;
    while(ros::ok())
    {

        mapping.waitForTransform();
        ++counter;
      //  mapping.broadcastTransform();
        mapping.updateGrid();
        if(counter % 100 == 0)
            mapping.publishMap();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
