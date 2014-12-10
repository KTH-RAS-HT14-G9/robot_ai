#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ir_converter/Distance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Header.h>
#include <common/robot.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <common/segmented_plane.h>
#include <navigation_msgs/Raycast.h>
#include <navigation_msgs/FitBlob.h>
#include <common/marker_delegate.h>
#include <navigation_msgs/TransformPoint.h>

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
    void wallDetectedCallback(const vision_msgs::Planes::ConstPtr&);
    bool performRaycast(navigation_msgs::RaycastRequest &request,
                        navigation_msgs::RaycastResponse &response);
    bool serviceFitRequest(navigation_msgs::FitBlobRequest& request,
                           navigation_msgs::FitBlobResponse& response);
    void activateUpdateCallback(const std_msgs::Bool::ConstPtr&);
    void updateGrid();
    void publishMap();
    void updateTransform();
    bool transformToRobot(navigation_msgs::TransformPointRequest &request, 
                                    navigation_msgs::TransformPointResponse &response);
    bool transformToMap(navigation_msgs::TransformPointRequest &request, 
                                    navigation_msgs::TransformPointResponse &response);

private:
    void markPointsBetween(Point<int> p0, Point<int> p1, double val);
    void updateIR(double ir_reading, double ir_x_offset);
    void markCellOccupied(Point<int> cell, int neighborhood = 1);
    void markPoint(Point<double> p, double val);
    Point<int> robotPointToCell(Point<double> p);
    Point<double> robotToMapTransform(Point<double> p);
    Point<int> mapPointToCell(Point<double> p);
    Point<double> transformPointToRobotSystem(std::string& frame_id, double x, double y);
    Point<int> transformPointToGridSystem(std::string& frame_id, double x, double y);
    Point<double> transformPointToMapSystem(std::string& frame_id, double x, double y);
    Point<double> transformCellToMap(Point<int>& cell);
    void markProbabilityGrid(Point<int> cell, double log_prob);
    void updateOccupancyGrid(Point<int>);
    bool isIRValid(double reading);
    void initProbabilityGrid();
    void initOccupancyGrid();
    void updateWalls();
    bool isObstacle(int x, int y);

    ros::NodeHandle handle;
    ros::Subscriber distance_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber wall_sub;
    ros::Subscriber object_sub;
    ros::Subscriber active_sub;
    ros::Publisher map_pub;
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

    ros::Publisher pub_viz;
    common::MarkerDelegate markers;

    ros::ServiceServer srv_raycast;
    ros::ServiceServer srv_fit;
    ros::ServiceServer srv_to_robot;
    ros::ServiceServer srv_to_map;

    common::vision::SegmentedPlane::ArrayPtr wall_planes;

    bool active;
    vector<vector<double> > prob_grid;
    nav_msgs::OccupancyGrid occupancy_grid;
    Point<double> pos;
    double fl_ir_reading, fr_ir_reading, bl_ir_reading, br_ir_reading;

    static const double INVALID_READING;
    static const double MAP_HEIGHT, MAP_WIDTH;
    static const int GRID_HEIGHT, GRID_WIDTH;
    static const double MAP_X_OFFSET, MAP_Y_OFFSET;
    static const double P_PRIOR, P_OCC, P_FREE;
    static const double FREE_OCCUPIED_THRESHOLD;

    static const double MAX_IR_DIST, MIN_IR_DIST;

    static const int UNKNOWN, FREE, OCCUPIED;
    static const int BLUE_CUBE;
    static const int RED_SPHERE;
  };

#endif // MAPPING_H
