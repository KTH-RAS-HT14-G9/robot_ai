#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <common/robot.h>
#include <common/parameter.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

//------------------------------------------------------------------------------
// Members

Parameter<double> _ir_theta_factor("/odometry/ir_theta_factor", 0.7);

double _x,_y,_theta, _ir_theta;
bool _turning;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

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

double ir_theta(double front, double rear, double front_offset, double rear_offset)
{
    if (front < 0.5 && rear < 0.5) {

        double y = front-rear;
        double x = front_offset+rear_offset;

        //if angle is greater than 45°
        if (std::abs(y) > x)
            return std::numeric_limits<double>::quiet_NaN();

        return std::atan(y/x);
    }

    return std::numeric_limits<double>::quiet_NaN();
}

void callback_ir(const ras_arduino_msgs::ADConverterConstPtr& adc)
{
    using namespace robot::ir;

    if (_turning)
        return;

    double fl = distance(id_front_left, adc->ch1) + offset_front_left;
    double fr = distance(id_front_right, adc->ch2) + offset_front_right;
    double bl = distance(id_rear_left, adc->ch3) + offset_rear_left;
    double br = distance(id_rear_right, adc->ch4) + offset_rear_right;

    double theta_left = ir_theta(fl,bl, offset_front_left_forward, offset_rear_left_forward);
    double theta_right = ir_theta(fr,br, offset_front_right_forward, offset_rear_right_forward);

    double theta_avg = 0;
    int n = 0;
    if (!std::isnan(theta_left)) {
        theta_avg += theta_left;
        n++;
    }
    if (!std::isnan(theta_right)) {
        theta_avg += theta_right;
        n++;
    }

    if (n > 0)
        _ir_theta = theta_avg /(double)n;
    else
        _ir_theta = std::numeric_limits<double>::quiet_NaN();
}

double fix_ir_direction(double ir_t) {
    int theta = (int)_theta;
    theta -= (theta%90);
    return theta+ir_t;
}

/**
  * Adapter from http://simreal.com/content/Odometry
  */
void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    static tf::TransformBroadcaster pub_tf;
    double dist_l = (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder1 / robot::prop::ticks_per_rev);
    double dist_r = (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder2 / robot::prop::ticks_per_rev);

    double theta = (dist_r - dist_l) / robot::dim::wheel_distance;

    if (!std::isnan(_ir_theta)) {
        double ir_theta = fix_ir_direction(_ir_theta);
        _theta = (1.0 - _ir_theta_factor())*(_theta+theta) + _ir_theta_factor()*ir_theta;
        _ir_theta = std::numeric_limits<double>::quiet_NaN();
    }
    else {
        _theta += theta;
    }

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

void callback_turn(const std_msgs::Float64ConstPtr& dummy) {
    _turning = true;
}

void callback_turn_done(const std_msgs::BoolConstPtr& dummy) {
    _turning = false;
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
    _ir_theta = std::numeric_limits<double>::quiet_NaN();
    _turning = false;

    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);
    ros::Subscriber sub_ir = n.subscribe("/arduino/adc",10,callback_ir);
    ros::Subscriber sub_turn = n.subscribe("/controller/turn/angle",10,callback_turn);
    ros::Subscriber sub_turn_done = n.subscribe("/controller/turn/done",10,callback_turn_done);

    _pub_odom = n.advertise<nav_msgs::Odometry>("/pose/odometry/",10,(ros::SubscriberStatusCallback)connect_callback);
    _pub_viz = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    ros::spin();

    return 0;
}
