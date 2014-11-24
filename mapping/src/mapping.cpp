#include "mapping/mapping.h"

const int Mapping::GRID_HEIGHT = 1000;
const int Mapping::GRID_WIDTH = 1000;
const int Mapping::GRID_X_OFFSET = GRID_WIDTH/2;
const int Mapping::GRID_Y_OFFSET = GRID_HEIGHT/2;

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
    fl_side(INVALID_READING), fr_side(INVALID_READING),
    bl_side(INVALID_READING), br_side(INVALID_READING),
    l_front(INVALID_READING), r_front(INVALID_READING)
{
    handle = ros::NodeHandle("");
    distance_sub = handle.subscribe("/perception/ir/distance", 10, &Mapping::distanceCallback, this);
    odometry_sub = handle.subscribe("/pose/odometry/", 10, &Mapping::odometryCallback, this);

    pc_pub = handle.advertise<PointCloud>("/mapping/point_cloud", 1);

    initProbGrid();
    initOccGrid();
    broadcastTransform();
}

void Mapping::broadcastTransform()
{
    tf_broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(MAP_X_OFFSET, MAP_Y_OFFSET, 0.0)),
                    ros::Time::now(),"map", "robot"));
}

Point2D<int> Mapping::posToCell(Point2D<double> pos)
{
    int x = round(pos.x*100.0);// + GRID_X_OFFSET;
    int y = round(pos.y*100.0);// + GRID_Y_OFFSET;
    // ROS_INFO("(%f, %f) ==> (%d, %d)", pos.x, pos.y, x, y);
    return Point2D<int>(x,y);
}

void Mapping::updateGrid()
{
    if(isReadingValid(fl_side))
        updateFL();

    if(isReadingValid(bl_side))
        updateBL();

    if(isReadingValid(fr_side))
        updateBL();

    if(isReadingValid(br_side))
        updateBR();

    // TODO update cells robot is at to free
}

bool Mapping::isReadingValid(double value)
{
    return value > 0.0 && value < 0.8;
}

void Mapping::updateFL()
{
    Point2D<double> fl_point = getPointPos(-1.0*robot::ir::offset_front_left, robot::ir::offset_front_left_forward, fl_side);
    Point2D<double> fl_sensor = getPointPos(-1.0*robot::ir::offset_front_left, robot::ir::offset_front_left_forward, 0.0);
    updateProbCell(posToCell(fl_point), P_OCC);
    updateOccCell(posToCell(fl_point));
    updateFreeCells(fl_sensor, fl_point);
}

void Mapping::updateFR()
{
    Point2D<double> fr_point = getPointPos(robot::ir::offset_front_right, robot::ir::offset_front_right_forward, fr_side);
    Point2D<double> fr_sensor = getPointPos(robot::ir::offset_front_right, robot::ir::offset_front_right_forward, 0.0);
    updateProbCell(posToCell(fr_point), P_OCC);
    updateOccCell(posToCell(fr_point));
    updateFreeCells(fr_sensor, fr_point);
}

void Mapping::updateBR()
{
    Point2D<double> br_point = getPointPos(robot::ir::offset_rear_right, -1.0*robot::ir::offset_rear_right_forward, br_side);
    Point2D<double> br_sensor = getPointPos(robot::ir::offset_rear_right, -1.0*robot::ir::offset_rear_right_forward, 0.0);
    updateProbCell(posToCell(br_point), P_OCC);
    updateOccCell(posToCell(br_point));
    updateFreeCells(br_sensor, br_point);
}

void Mapping::updateBL()
{
    Point2D<double> bl_point = getPointPos(-1.0*robot::ir::offset_rear_left, -1.0*robot::ir::offset_rear_left_forward, bl_side);
    Point2D<double> bl_sensor = getPointPos(-1.0*robot::ir::offset_rear_left, -1.0*robot::ir::offset_rear_left_forward, 0.0);
    updateProbCell(posToCell(bl_point), P_OCC);
    updateOccCell(posToCell(bl_point));
    updateFreeCells(bl_sensor, bl_point);
}

Point2D<double> Mapping::getPointPos(double side_offset, double forward_offset, double ir_distance)
{
    geometry_msgs::PointStamped robot_point;
    robot_point.header.frame_id = "robot";
    robot_point.header.stamp = ros::Time();
    robot_point.point.x = pos.x + side_offset + ir_distance;
    robot_point.point.y = pos.y + forward_offset;
    robot_point.point.z = 0.0;

    geometry_msgs::PointStamped map_point;
    tf_listener.transformPoint("map", robot_point, map_point);
    return Point2D<double>(map_point.point.x, map_point.point.y);
}

void Mapping::updateFreeCells(Point2D<double> sensor, Point2D<double> obstacle)
{
    double min_x = std::min(sensor.x, obstacle.x);
    double max_x = std::max(sensor.x, obstacle.x);
    double min_y = std::min(sensor.y, obstacle.y);
    double max_y = std::max(sensor.y, obstacle.y);

    double x1 = min_x;
    double x2 = max_x;
    double y1 = sensor.x == x1 ? sensor.y : obstacle.y;
    double y2 = sensor.x == x2 ? sensor.y : obstacle.y;
    double dx = x2-x1;
    double dy = y2-y1;


  //  ROS_INFO("sensor: %f, %f, obstacle: %f, %f", sensor.x, sensor.y, obstacle.x, obstacle.y);
 //   ROS_INFO("min x %f, y %f, max x %f, y %f", min_x, min_y, max_x, max_y);

    if(dx < 0.01)
    {
        double x = x1;
        for(double y = min_y; y < max_y; y +=0.01)
        {
            Point2D<int> cell = posToCell(Point2D<double>(x,y));
      //      ROS_INFO("Marking: (%d,%d)", cell.x, cell.y);
            updateProbCell(cell, P_FREE);
            updateOccCell(cell);
        }
    } else {

        for(double x = x1; x < x2; x+=0.01)
        {
            double y = y1 + dy * (x-x1) / dx;
            Point2D<int> cell = posToCell(Point2D<double>(x,y));
        //    ROS_INFO("Marking: (%d,%d)", cell.x, cell.y);
            updateProbCell(cell, P_FREE);
            updateOccCell(cell);
        }
    }

    //ROS_INFO("done");
   // ros::Duration(10.0).sleep();
}

void Mapping::updateProbCell(Point2D<int> cell, double p)
{
    prob_grid[cell.x][cell.y] += p - P_PRIOR;
}

void Mapping::updateOccCell(Point2D<int> cell)
{
    if(prob_grid[cell.x][cell.y] > FREE_OCCUPIED_THRESHOLD)
        occ_grid[cell.x][cell.y] = OCCUPIED;
    else if(prob_grid[cell.x][cell.y] < FREE_OCCUPIED_THRESHOLD)
        occ_grid[cell.x][cell.y] = FREE;
    else
        occ_grid[cell.x][cell.y] = UNKNOWN;

}

void Mapping::initProbGrid()
{
    prob_grid.resize(GRID_WIDTH);
    for(int x = 0; x < GRID_WIDTH; ++x)
        prob_grid[x].resize(GRID_HEIGHT);

    for(int x = 0; x < GRID_WIDTH; ++x)
        for(int y = 0; y < GRID_HEIGHT; ++y)
            prob_grid[x][y] = P_PRIOR;
}

void Mapping::initOccGrid()
{
    occ_grid.resize(GRID_WIDTH);
    for(int x = 0; x < GRID_WIDTH; ++x)
        occ_grid[x].resize(GRID_HEIGHT);

    for(int x = 0; x < GRID_WIDTH; ++x)
        for(int y = 0; y < GRID_HEIGHT; ++y)
            occ_grid[x][y] = UNKNOWN;
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
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    pos = Point2D<double>(x,y);
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
        mapping.broadcastTransform();
        mapping.updateGrid();
        if(counter % 100 == 0)
            mapping.publishMap();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
