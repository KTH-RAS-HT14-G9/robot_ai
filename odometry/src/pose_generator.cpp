#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ir_converter/Distance.h>
#include <common/parameter.h>

#define RAD2DEG(x) ((x)*M_PI/180.0)
#define DEG2RAD(x) ((x)*180.0/M_PI)

//------------------------------------------------------------------------------
// Members

Parameter<int> _max_iterations("/pose/odometry/num_correction_iterations",5);

double _x,_y,_theta;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

//ir_converter::Distance _ir_dist;
Eigen::Matrix<double,4,1> _ir_dist;
bool _correct_theta;
int _iteration;
double _turn_accum;
int _heading;

ros::Publisher _pub_odom;
ros::Publisher _pub_viz;

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
    _robot_marker.color.a = 1.0;
    _robot_marker.color.r = 0.0;
    _robot_marker.color.g = 141.0 / 255.0;
    _robot_marker.color.b = 240.0 / 255.0;

    _pub_viz.publish(_robot_marker);
}

/**
  * Adapter from http://simreal.com/content/Odometry
  */
void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    static tf::TransformBroadcaster pub_tf;

    double c_l = 1.0;//0.98416;
    double c_r = 1.0;//1.0538;

    double dist_l = c_l * (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder1 / robot::prop::ticks_per_rev);
    double dist_r = c_r * (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder2 / robot::prop::ticks_per_rev);

    _theta += (dist_r - dist_l) / robot::dim::wheel_distance;

    double dist = (dist_r + dist_l) / 2.0;

    _x += dist * cos(_theta);
    _y += dist * sin(_theta);

    pack_pose(_q, _odom);
    _pub_odom.publish(_odom);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_x, _y, 0));
    transform.setRotation(_q);
    pub_tf.sendTransform(tf::StampedTransform(transform, _odom.header.stamp, "map", "robot"));

    send_marker(transform);
}

void connect_callback(const ros::SingleSubscriberPublisher& pub)
{
    pack_pose(_q,_odom);
    pub.publish(_odom);
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
        return m(0,0) - m(1,0);
    else if (wall_close(m(2,0), m(3,0)))
        return m(2,0) - m(3,0);
    else
        return std::numeric_limits<double>::quiet_NaN();
}

void callback_ir(const ir_converter::DistanceConstPtr& distances)
{
    if (_correct_theta)
    {
        _ir_dist += pack_matrix(distances);
        std::cout << "Distances: " << _ir_dist << std::endl;
        ++_iteration;

        if (_iteration >= _max_iterations()) {

            _ir_dist /= _iteration;
            std::cout << "Distances div: " << _ir_dist << std::endl;

            double dx = robot::ir::offset_front_left_forward + robot::ir::offset_rear_left_forward;
            double dy = get_dy(_ir_dist);

            ROS_ERROR("dy = %lf",dy);

            if(isnan(dy))
                return;

            double angle = atan(dy/dx);

            double new_theta = (_heading*M_PI_2) + angle;

            ROS_ERROR("corrected theta %.3lf -> %.3lf", RAD2DEG(_theta), RAD2DEG(new_theta));
            _theta = new_theta;

            _correct_theta = false;
            _iteration = 0;
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

    if (std::abs(angle) < M_PI_4) {
        ROS_INFO("Attempt to correct theta by using ir sensors");
        _correct_theta = true;
    }
}

void callback_turn_angle(const std_msgs::Float64ConstPtr& angle)
{
    _turn_accum += angle->data;
    while (_turn_accum >= 90.0) {
        _turn_accum -= 90.0;
        _heading++;
    }

    while (_turn_accum <= -90.0) {
        _turn_accum += 90.0;
        _heading--;
    }

    _heading = ( (_heading+2)%4 ) - 2;
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_generator");

    ros::NodeHandle n;

    _odom.header.frame_id = "map";
    _x = _y = 0;
    _theta = 0;
    _correct_theta = false;
    _iteration = 0;
    _turn_accum = 0;
    _heading = 0;

    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);
    ros::Subscriber sub_turn_angle = n.subscribe("/controller/turn/angle",10,callback_turn_angle);
    ros::Subscriber sub_turn_done = n.subscribe("/controller/turn/done",10,callback_turn_done);
    ros::Subscriber sub_ir = n.subscribe("/perception/ir/distance",10,callback_ir);
    _pub_odom = n.advertise<nav_msgs::Odometry>("/pose/odometry/",10,(ros::SubscriberStatusCallback)connect_callback);
    _pub_viz = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    ros::spin();

    return 0;
}
