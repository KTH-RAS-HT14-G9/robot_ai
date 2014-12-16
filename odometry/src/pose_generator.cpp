#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Time.h>
#include <ir_converter/Distance.h>
#include <common/parameter.h>
#include <navigation_msgs/Raycast.h>
#include <vision_msgs/Planes.h>

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)

//------------------------------------------------------------------------------
// Members

Parameter<int> _max_iterations("/pose/odometry/num_correction_iterations",5);
Parameter<bool> _enable_lateral_correction("/pose/odometry/correction/lateral_enabled",false);
Parameter<bool> _enable_theta_correction("/pose/odometry/correction/theta_enabled",false);
Parameter<int> _revert_last_msec("/pose/odometry/revert_last_msec",100);

ros::NodeHandlePtr _handle;
ros::Timer _timer;

double _x,_y,_theta;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

//ir_converter::Distance _ir_dist;
Eigen::Matrix<double,4,1> _ir_dist;
bool _correct_theta, _correct_lateral;
int _iteration_theta, _iteration_lateral;
double _turn_accum;
int _heading;

bool _see_front_plane;
vision_msgs::Plane _front_plane;

ros::Publisher _pub_odom;
ros::Publisher _pub_viz;
ros::Publisher _pub_compass;
ros::ServiceClient _srv_raycast;

std::vector<ras_arduino_msgs::Encoders> _ringbuffer;
int _ringbuffer_max_size = 50*2;

bool _mute = false;
double _muting_time = 1.0;

//------------------------------------------------------------------------------
// Methods

/**
  * Packs current state in a odom message. Needs a quaternion for conversion.
  */
void pack_pose(tf::Quaternion& q, nav_msgs::Odometry& odom)
{
    q.setRPY(0, 0, _theta);

    odom.header.stamp = ros::Time::now();

    odom.pose.pose.position.x = _x;
    odom.pose.pose.position.y = _y;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
}

//------------------------------------------------------------------------------
// Callbacks

void callback_timer(const ros::TimerEvent& event)
{
    _mute = false;
    ROS_ERROR("[PoseGenerator::callback_timer] Enabling encoder readings");
}

void revert_applied_readings_since(const ros::Time& time)
{
    int msec = (int)(time.toNSec()/1000l) - _revert_last_msec();

    int k = 0;

    for(std::vector<ras_arduino_msgs::Encoders>::reverse_iterator it = _ringbuffer.rbegin(); it != _ringbuffer.rend(); ++it)
    {
        ras_arduino_msgs::Encoders& enc = *it;
        if (enc.timestamp >= msec)
        {
            ++k;

            double dist_l = (2.0*M_PI*robot::dim::wheel_radius) * (enc.delta_encoder1 / robot::prop::ticks_per_rev);
            double dist_r = (2.0*M_PI*robot::dim::wheel_radius) * (enc.delta_encoder2 / robot::prop::ticks_per_rev);

            _theta += (dist_r - dist_l) / robot::dim::wheel_distance;

            double dist = (dist_r + dist_l) / 2.0;

            _x += dist * cos(_theta);
            _y += dist * sin(_theta);

            //remove reading
            _ringbuffer.erase(--it.base());
        }
    }

    ROS_ERROR("[PoseGenerator::revertReadingsSince] Reverted %d readings.",k);
}

void callback_crash(const std_msgs::TimeConstPtr& time)
{
    _mute = true;

    revert_applied_readings_since(time->data);
    _timer = _handle->createTimer(ros::Duration(_muting_time), callback_timer, true, true );

    ROS_ERROR("[PoseGenerator::callbackCrash] Crash signal received. Will mute encoder readings for %.2lf seconds", _muting_time);
}

void send_marker(tf::Transform& transform) {
    visualization_msgs::Marker _robot_marker;
    _robot_marker.header.frame_id = "robot";
    _robot_marker.header.stamp = _odom.header.stamp;
    _robot_marker.ns = "robot";
    _robot_marker.id = 0;
    _robot_marker.type = visualization_msgs::Marker::CUBE;
    _robot_marker.action = visualization_msgs::Marker::ADD;
    _robot_marker.pose.position.x = 0;
    _robot_marker.pose.position.y = 0;
    _robot_marker.pose.position.z = robot::dim::robot_height/2.0;
    _robot_marker.pose.orientation.x = 0;
    _robot_marker.pose.orientation.y = 0;
    _robot_marker.pose.orientation.z = 0;
    _robot_marker.pose.orientation.w = 1;
    _robot_marker.scale.x = robot::dim::wheel_distance;
    _robot_marker.scale.y = robot::dim::wheel_distance;
    _robot_marker.scale.z = robot::dim::robot_height;
    _robot_marker.color.a = 0.5;
    _robot_marker.color.r = 0.0;
    _robot_marker.color.g = 141.0 / 255.0;
    _robot_marker.color.b = 240.0 / 255.0;

    _pub_viz.publish(_robot_marker);
}

void ringbuffer_push(const ras_arduino_msgs::Encoders& encoders)
{
    if (_ringbuffer.size() >= _ringbuffer_max_size)
    {
        _ringbuffer.erase(_ringbuffer.begin());
    }
    _ringbuffer.push_back(encoders);
}

int get_compass()
{
    /**
      * -1 -> S -> 2
      *  0 -> E -> 1
      *  1 -> N -> 0
      *  2 -> W -> 3
      */

    switch(_heading+1) {
    case 0: return 2;
    case 1: return 1;
    case 2: return 0;
    case 3: return 3;
    }

    ROS_ERROR("[pose_generator::get_compass] Unexpected heading %d", _heading);
    return -1;
}

void update_heading(double dTheta)
{
        std::stringstream ss;
    //    ss << "Turning by angle: " << angle->data << ". Accum= " << _turn_accum << ", Heading= " << _heading;

        _turn_accum += RAD2DEG(dTheta);

        int prev_heading = _heading;

        while (_turn_accum > 45.0) {
            _turn_accum -= 90.0;
            _heading++;
            if (_heading >= 3) _heading -= 4;
        }

        while (_turn_accum < -45.0) {
            _turn_accum += 90.0;
            _heading--;
            if (_heading <= -2) _heading += 4;
        }

//        ss << "Corrected heading = " << _heading << ", new accum= " << _turn_accum << std::endl;
//        ROS_ERROR("%s",ss.str().c_str());

        if (_heading != prev_heading) {
            std_msgs::Int8 msg;
            msg.data = get_compass();
            _pub_compass.publish(msg);
        }
}

/**
  * Adapter from http://simreal.com/content/Odometry
  */
void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    static tf::TransformBroadcaster pub_tf;

    if (!_mute) {

        double c_l = 1.0;//0.98416;
        double c_r = 1.0;//1.0538;

        double dist_l = c_l * (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder1 / robot::prop::ticks_per_rev);
        double dist_r = c_r * (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder2 / robot::prop::ticks_per_rev);

        double dTheta = (dist_r - dist_l) / robot::dim::wheel_distance;
        update_heading(dTheta);

        _theta += dTheta;

        double dist = (dist_r + dist_l) / 2.0;

        _x += dist * cos(_theta);
        _y += dist * sin(_theta);

        ras_arduino_msgs::Encoders fixed = *encoders;
        fixed.timestamp = (int)(ros::Time::now().toNSec()/1000l);
        ringbuffer_push(fixed);
    }

    pack_pose(_q, _odom);
    _pub_odom.publish(_odom);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_x, _y, 0));
    transform.setRotation(_q);
    pub_tf.sendTransform(tf::StampedTransform(transform, _odom.header.stamp, "map", "robot"));

    send_marker(transform);
}

void connect_odometry_callback(const ros::SingleSubscriberPublisher& pub)
{
    pack_pose(_q,_odom);
    pub.publish(_odom);
}

void connect_compass_callback(const ros::SingleSubscriberPublisher& pub)
{
    std_msgs::Int8 msg;
    msg.data = get_compass();
    pub.publish(msg);
}

Eigen::Matrix<double, 4, 1> pack_matrix(const ir_converter::DistanceConstPtr& distances)
{
    Eigen::Matrix<double, 4, 1> m;
    m(0,0) = distances->fl_side;
    m(1,0) = distances->bl_side;
    m(2,0) = distances->fr_side;
    m(3,0) = distances->br_side;

    return m;
}

bool wall_close(double front, double rear) {
    return front < 0.5 && rear < 0.5;
}

double get_dy(Eigen::Matrix<double,4,1>& m)
{
    if (wall_close(m(0,0), m(1,0) ))
        return -(m(0,0) - m(1,0));
    else if (wall_close(m(2,0), m(3,0)))
        return m(2,0) - m(3,0);
    else
        return std::numeric_limits<double>::quiet_NaN();
}

bool request_raycast(double x, double y, double dir_x, double dir_y, double& dist)
{
    navigation_msgs::RaycastRequest request;
    navigation_msgs::RaycastResponse response;

    request.frame_id = "robot";
    request.origin_x = x;
    request.origin_y = y;
    request.dir_x = dir_x;
    request.dir_y = dir_y;
    request.max_length = 0.8;

    response.hit = false;
    _srv_raycast.call(request, response);

    if (response.hit) {
        dist = response.hit_dist;
        return true;
    }

    return false;
}

bool request_raycast_fl(double ir_dist, double& dist)
{
    double x = robot::ir::offset_front_left_forward;
    double y = 0;

    double dir_x = 0;
    double dir_y = 1;

    return request_raycast(x,y, dir_x,dir_y, dist);
}

bool request_raycast_fr(double ir_dist, double& dist)
{
    double x = robot::ir::offset_front_right_forward;
    double y = 0;

    double dir_x = 0;
    double dir_y = -1;

    return request_raycast(x,y, dir_x,dir_y, dist);
}

bool request_raycast_bl(double ir_dist, double& dist)
{
    double x = -robot::ir::offset_rear_left_forward;
    double y = 0;

    double dir_x = 0;
    double dir_y = 1;

    return request_raycast(x,y, dir_x,dir_y, dist);
}

bool request_raycast_br(double ir_dist, double& dist)
{
    double x = -robot::ir::offset_rear_right_forward;
    double y = 0;

    double dir_x = 0;
    double dir_y = -1;

    return request_raycast(x,y, dir_x,dir_y, dist);
}

double get_y_diff(Eigen::Matrix<double,4,1>& m)
{
    if (wall_close(m(0,0), m(1,0) )) {
        double fl_dist, bl_dist;
        if (!request_raycast_fl(m(0,0), fl_dist))
            return std::numeric_limits<double>::quiet_NaN();

        if (!request_raycast_bl(m(1,0), bl_dist))
            return std::numeric_limits<double>::quiet_NaN();

        double avg_ray = (fl_dist+bl_dist)/2.0;
        double avg_ir = (m(0,0) + m(1,0))/2.0;

        return avg_ir - avg_ray;
    }
    else if (wall_close(m(2,0), m(3,0))) {
        double fr_dist, br_dist;
        if (!request_raycast_fl(m(2,0), fr_dist))
            return std::numeric_limits<double>::quiet_NaN();

        if (!request_raycast_bl(m(3,0), br_dist))
            return std::numeric_limits<double>::quiet_NaN();

        double avg_ray = (fr_dist+br_dist)/2.0;
        double avg_ir = (m(2,0) + m(3,0))/2.0;

        return avg_ir - avg_ray;
    }
    else
        return std::numeric_limits<double>::quiet_NaN();
}

double get_x_diff()
{
    if (_see_front_plane)
    {
        double dist_to_plane = _front_plane.bounding_box[0]/*x*/;
        double dist_to_obstacle;
        if (request_raycast(0,0,1,0,dist_to_obstacle))
        {
            ROS_ERROR("Dist to obstacle: %.3lf, to plane: %.3lf",dist_to_obstacle, dist_to_plane);
            return dist_to_obstacle - dist_to_plane;
        }
        else {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }
    else
        return std::numeric_limits<double>::quiet_NaN();
}

void callback_ir(const ir_converter::DistanceConstPtr& distances)
{
    if (_correct_theta)
    {
        _ir_dist += pack_matrix(distances);
        ++_iteration_theta;

        if (_iteration_theta >= _max_iterations()) {

            _ir_dist /= _iteration_theta;

            double dx = robot::ir::offset_front_left_forward + robot::ir::offset_rear_left_forward;
            double dy = get_dy(_ir_dist);

            if(isnan(dy))
                return;

            double angle = atan(dy/dx);

            double new_theta = (_heading*M_PI_2) + angle;

            ROS_INFO("corrected theta %.3lf -> %.3lf", RAD2DEG(_theta), RAD2DEG(new_theta));
            _theta = new_theta;

            _correct_theta = false;
            _iteration_theta = 0;
        }
    }
    else
    {
        _ir_dist = pack_matrix(distances);
    }
}

void callback_turn_done(const std_msgs::BoolConstPtr& done)
{
    double dx = robot::ir::offset_front_left_forward + robot::ir::offset_rear_left_forward;
    double dy = get_dy(_ir_dist);

    if(isnan(dy))
        return;

    double angle = atan(dy/dx);

    if (RAD2DEG(std::abs(angle)) < 10.0 && std::abs(_turn_accum) < 20.0) {
        ROS_INFO("Attempt to correct theta by using ir sensors");
        _correct_theta = _enable_theta_correction();
        _correct_lateral = _enable_lateral_correction();
    }

}

void callback_turn_angle(const std_msgs::Float64ConstPtr& angle)
{

}

double _avg_plane_dist;
int _accumulated_plane_dists;
void callback_planes(const vision_msgs::PlanesConstPtr& planes)
{
    if (!_correct_lateral) {
        _avg_plane_dist = 0;
        _accumulated_plane_dists = 0;
        return;
    }

    //find wall that is perpendicular to the robots direction
    double max_dot = 0.0;
    int ortho_plane = 0;
    for(int i = 0; i < planes->planes.size(); ++i)
    {
        const vision_msgs::Plane& plane = planes->planes[i];
        if (plane.is_ground_plane) continue;

        Eigen::Vector3f plane_normal(plane.plane_coefficients[0],
                                     plane.plane_coefficients[1],
                                     plane.plane_coefficients[2]);
        Eigen::Vector3f forward(1,0,0);

        double dotprod = std::abs(forward.dot(plane_normal));
        if (dotprod > max_dot) {
            max_dot = dotprod;
            ortho_plane = i;
        }
    }

    if (max_dot > 0.9) {
        _front_plane = planes->planes[ortho_plane];
        _see_front_plane = true;

        _iteration_lateral++;

        double x_diff = get_x_diff();
        ROS_ERROR("x diff = %.3lf",x_diff);

        if (!std::isnan(x_diff)) {
            _avg_plane_dist += x_diff;
            _accumulated_plane_dists++;
        }

        if (_iteration_lateral >= _max_iterations()) {

            if (_accumulated_plane_dists > 0) {

                double avg_diff = _avg_plane_dist / (double)_accumulated_plane_dists;

                ROS_ERROR("Attempt to correct position based on wall");

                if (std::abs(avg_diff) < 0.1) {
                    double dx = cos(_theta);
                    double dy = sin(_theta);

                    double new_x = _x + avg_diff*dx;
                    double new_y = _y + avg_diff*dy;

                    ROS_ERROR("corrected position (%.3lf,%.3lf) -> (%.3lf,%.3lf)", _x, _y, new_x, new_y);

                    _x = new_x;
                    _y = new_y;
                }
            }

            _iteration_lateral = 0;
            _avg_plane_dist = 0;
            _accumulated_plane_dists = 0;
            _correct_lateral = false;
        }
    }
    else {
        _see_front_plane = false;

        if (_correct_lateral) _correct_lateral = false;
    }
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_generator");

    _handle = ros::NodeHandlePtr(new ros::NodeHandle(""));

    _odom.header.frame_id = "map";
    _x = _y = 0;
    _theta = 0;
    _correct_theta = false;
    _correct_lateral = false;
    _iteration_theta = 0; _iteration_lateral = 0;
    _turn_accum = 0;
    _heading = 0;

    _see_front_plane = false;

    _ringbuffer.reserve(_ringbuffer_max_size);

    ros::Subscriber sub_enc = _handle->subscribe("/arduino/encoders",10,callback_encoders);
    ros::Subscriber sub_turn_angle = _handle->subscribe("/controller/turn/angle",10,callback_turn_angle);
    ros::Subscriber sub_turn_done = _handle->subscribe("/controller/turn/done",10,callback_turn_done);
    ros::Subscriber sub_ir = _handle->subscribe("/perception/ir/distance",10,callback_ir);
    ros::Subscriber sub_planes = _handle->subscribe("/vision/obstacles/planes",10,callback_planes);
    ros::Subscriber sub_crash = _handle->subscribe("/perception/imu/peak", 10, callback_crash);

    _pub_odom = _handle->advertise<nav_msgs::Odometry>("/pose/odometry/",10,(ros::SubscriberStatusCallback)connect_odometry_callback);
    _pub_viz = _handle->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    _pub_compass = _handle->advertise<std_msgs::Int8>("/pose/compass", 10, (ros::SubscriberStatusCallback)connect_compass_callback);

    _srv_raycast = _handle->serviceClient<navigation_msgs::Raycast>("/mapping/raycast");

    ros::spin();

    return 0;
}
