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
#include <common/parameter.h>
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
#include <navigation_msgs/UnexploredRegion.h>
#include <common/marker_delegate.h>
#include <navigation_msgs/TransformPoint.h>
#include <fstream>

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
    void activateUpdateCallback(const std_msgs::Bool::ConstPtr&);
    void saveMapCallback(const std_msgs::Empty::ConstPtr&);
    bool performRaycast(navigation_msgs::RaycastRequest &request,
                        navigation_msgs::RaycastResponse &response);
    bool serviceFitRequest(navigation_msgs::FitBlobRequest& request,
                           navigation_msgs::FitBlobResponse& response);
    bool serviceHasUnexploredRegion(navigation_msgs::UnexploredRegionRequest& request,
                                    navigation_msgs::UnexploredRegionResponse& response);
    void updateGrid();
    void publishMap();
    void updateTransform();
    bool transformToRobot(navigation_msgs::TransformPointRequest &request, 
                                    navigation_msgs::TransformPointResponse &response);
    bool transformToMap(navigation_msgs::TransformPointRequest &request, 
                                    navigation_msgs::TransformPointResponse &response);
    void saveToFile(const std::string& file_name);
    void recoverFromFile(const std::string& file_name);
    void recoverAndRefreshOccGrid(const std::string& file_name);

private:

    void markPointsBetween(Point<int> p0, Point<int> p1, double val, bool markInSeen = false);
    void updateIR(double ir_reading, double ir_x_offset);
    void updateHaveSeen();
    void markCellOccupied(Point<int> cell, int neighborhood = 1);
    void markPoint(Point<double> p, double val);
    Point<int> robotPointToCell(Point<double> p);
    Point<double> robotToMapTransform(Point<double> p);
    Point<int> mapPointToCell(Point<double> p);
    Point<double> transformPointToRobotSystem(std::string& frame_id, double x, double y);
    Point<double> transformPointToMapSystem(std::string& frame_id, double x, double y);
    Point<int> transformPointToGridSystem(const std::string &frame_id, double x, double y);
    Point<double> transformCellToMap(Point<int>& cell);
    void markProbabilityGrid(Point<int> cell, double log_prob);
    void markSeenGrid(Point<int> cell, int flag);
    void updateOccupancyGrid(Point<int>);
    void updateSeenVizGrid(Point<int>);
    bool isIRValid(double reading);
    void initProbabilityGrid();
    void initOccupancyGrid();
    void updateWalls(bool markOnHaveSeen);
    bool isObstacle(int x, int y, bool inHaveSeen = false);
    bool isUnexplored(int x, int y);

    ros::NodeHandle handle;
    ros::Subscriber distance_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber wall_sub;
    ros::Subscriber object_sub;
    ros::Subscriber active_sub;
    ros::Subscriber map_save;

    ros::Publisher map_pub;
    ros::Publisher seen_pub;

    Parameter<double> frustum_fov;
    Parameter<double> frustum_dist;
    Parameter<bool> use_planes;

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

    ros::Publisher pub_viz;
    common::MarkerDelegate markers_map;
    common::MarkerDelegate markers_robot;

    ros::ServiceServer srv_raycast;
    ros::ServiceServer srv_fit;
    ros::ServiceServer srv_to_robot;
    ros::ServiceServer srv_to_map;
    ros::ServiceServer srv_isunexplored;

    common::vision::SegmentedPlane::ArrayPtr wall_planes;

    vector<vector<double> > prob_grid;
    vector<vector<uint8_t> > seen_grid;
    nav_msgs::OccupancyGrid occupancy_grid;

    nav_msgs::OccupancyGrid seen_viz_grid;

    Point<double> pos;
    double fl_ir_reading, fr_ir_reading, bl_ir_reading, br_ir_reading;
    bool active;

    static const double INVALID_READING;
    static const double MAP_HEIGHT, MAP_WIDTH;
    static const int GRID_HEIGHT, GRID_WIDTH;
    static const double MAP_X_OFFSET, MAP_Y_OFFSET;
    static const double P_PRIOR, P_OCC, P_FREE;
    static const double FREE_OCCUPIED_THRESHOLD;
    static const std::string MAP_NAME;

    static const double MAX_IR_DIST, MIN_IR_DIST;

    static const int UNKNOWN, FREE, OCCUPIED;
    static const int BLUE_CUBE;
    static const int RED_SPHERE;
  };

#endif // MAPPING_H
