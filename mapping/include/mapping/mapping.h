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
    void wallDetectedCallback(const vision_msgs::Planes::ConstPtr&);
    void activateUpdateCallback(const std::msgs::Bool::ConstPtr&);
    bool performRaycast(navigation_msgs::RaycastRequest &request,
                        navigation_msgs::RaycastResponse &response);
    bool serviceFitRequest(navigation_msgs::FitBlobRequest& request,
                           navigation_msgs::FitBlobResponse& response);
    void updateGrid();
    void publishMap();
    void updateTransform();

private:
    //void markPointsBetween(Point<double> p1, Point<double> p2, double val);
    void markPointsBetween(Point<int> p0, Point<int> p1, double val);
    void updateIR(double ir_reading, double ir_x_offset);
    //void markPointOccupied(Point<double> p);
    void markCellOccupied(Point<int> cell, int neighborhood = 1);
    void markPoint(Point<double> p, double val);
    Point<int> robotPointToCell(Point<double> p);
    Point<double> robotToMapTransform(Point<double> p);
    Point<int> mapPointToCell(Point<double> p);
    Point<double> transformPointToRobotSystem(std::string& frame_id, double x, double y);
    Point<int> transformPointToGridSystem(std::string& frame_id, double x, double y);
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
    ros::Subscriber start_turn_sub;
    ros::Subscriber stop_turn_sub;
    ros::Subscriber wall_sub;
    ros::Subscriber object_sub;
    ros::Publisher map_pub;
    ros::Subscriber active_sub;
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

    ros::Publisher pub_viz;
    common::MarkerDelegate markers;

    ros::ServiceServer srv_raycast;
    ros::ServiceServer srv_fit;

    common::vision::SegmentedPlane::ArrayPtr wall_planes;

    vector<vector<double> > prob_grid;
    nav_msgs::OccupancyGrid occupancy_grid;
    bool turning;
    Point<double> pos;
    double fl_ir_reading, fr_ir_reading, bl_ir_reading, br_ir_reading;
    bool active;

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
