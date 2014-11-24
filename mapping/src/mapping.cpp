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
    pos(Point<double>(0.0,0.0))
{
    handle = ros::NodeHandle("");
    distance_sub = handle.subscribe("/perception/ir/distance", 10, &Mapping::distanceCallback, this);
    odometry_sub = handle.subscribe("/pose/odometry/", 10, &Mapping::odometryCallback, this);

    pc_pub = handle.advertise<PointCloud>("/mapping/point_cloud", 1);

    initProbabilityGrid();
    initOccupancyGrid();
    broadcastTransform();
}

void Mapping::updateGrid()
{
    if(isIRValid(fl_ir))
    {
        double ir_x_offset = -1.0*robot::ir::offset_front_left;
        double ir_y_offset = robot::ir::offset_front_left_forward;
        updateIR(Point<double>(pos.x + ir_x_offset, pos.y + ir_y_offset), fl_ir);
    }

    if(isIRValid(fr_ir))
    {
        double ir_x_offset = robot::ir::offset_front_left;
        double ir_y_offset = robot::ir::offset_front_left_forward;
        updateIR(Point<double>(pos.x + ir_x_offset, pos.y + ir_y_offset), fr_ir);
    }

    if(isIRValid(bl_ir))
    {
        double ir_x_offset = -1.0*robot::ir::offset_front_left;
        double ir_y_offset = -1.0*robot::ir::offset_front_left_forward;
        updateIR(Point<double>(pos.x + ir_x_offset, pos.y + ir_y_offset), bl_ir);
    }

    if(isIRValid(br_ir))
    {
        double ir_x_offset = -1.0*robot::ir::offset_front_left;
        double ir_y_offset = robot::ir::offset_front_left_forward;
        updateIR(Point<double>(pos.x + ir_x_offset, pos.y + ir_y_offset), br_ir);
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

void Mapping::updateIR(Point<double> ir_pos, double ir_reading)
{
    Point<double> obstacle_pos = Point<double>(ir_pos.x + ir_reading , ir_pos.y);
    markPointOccupied(obstacle_pos);
    markPointsFreeBetween(ir_pos, obstacle_pos);
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
    robot_point.header.stamp = ros::Time();
    robot_point.point.x = point.x;
    robot_point.point.y = point.y;
    robot_point.point.z = 0.0;

    geometry_msgs::PointStamped map_point;
    tf_listener.transformPoint("map", robot_point, map_point);
    return Point<double>(map_point.point.x+MAP_X_OFFSET, map_point.point.y+MAP_Y_OFFSET);
}

Point<int> Mapping::mapPointToCell(Point<double> pos)
{
    int x = round(pos.x*100.0);
    int y = round(pos.y*100.0);
    return Point<int>(x,y);
}

void Mapping::markProbabilityGrid(Point<int> cell, double log_prob)
{
    prob_grid[cell.x][cell.y] += log_prob - P_PRIOR;
}

void Mapping::updateOccupancyGrid(Point<int> cell)
{
    if(prob_grid[cell.x][cell.y] > FREE_OCCUPIED_THRESHOLD)
        occ_grid[cell.x][cell.y] = OCCUPIED;
    else if(prob_grid[cell.x][cell.y] < FREE_OCCUPIED_THRESHOLD)
        occ_grid[cell.x][cell.y] = FREE;
    else
        occ_grid[cell.x][cell.y] = UNKNOWN;
}

bool Mapping::isIRValid(double value)
{
    return value > 0.0 && value < 0.8;
}

void Mapping::initProbabilityGrid()
{
    prob_grid.resize(GRID_WIDTH);
    for(int x = 0; x < GRID_WIDTH; ++x)
        prob_grid[x].resize(GRID_HEIGHT);

    for(int x = 0; x < GRID_WIDTH; ++x)
        for(int y = 0; y < GRID_HEIGHT; ++y)
            prob_grid[x][y] = P_PRIOR;
}

void Mapping::initOccupancyGrid()
{
    occ_grid.resize(GRID_WIDTH);
    for(int x = 0; x < GRID_WIDTH; ++x)
        occ_grid[x].resize(GRID_HEIGHT);

    for(int x = 0; x < GRID_WIDTH; ++x)
        for(int y = 0; y < GRID_HEIGHT; ++y)
            occ_grid[x][y] = UNKNOWN;
}

void Mapping::broadcastTransform()
{
    tf_broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                    ros::Time::now(),"map", "robot"));
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

void Mapping::publishMap()
{
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "map";
    msg->height = GRID_HEIGHT;
    msg->width = GRID_WIDTH;

    int numFree = 0;
    int numUnknown = 0;
    int numOcc = 0;

    for(int x = 0; x < GRID_WIDTH; ++x)
    {
        for(int y = 0; y < GRID_HEIGHT; ++y)
        {
            int cell = occ_grid[x][y];

            if(cell == OCCUPIED)
                ++numOcc;
            else if(cell == FREE)
                ++numFree;
            else if(cell == UNKNOWN)
                ++numUnknown;

            double i = (cell == FREE) ? 0.0 : (cell == OCCUPIED) ? 1.0 : 0.5;

            PCPoint p(i);
            p.x = (double) x/100.0;
            p.y = (double) y/100.0;
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
        ++counter;

        mapping.updateGrid();
        if(counter % 100 == 0)
            mapping.publishMap();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
