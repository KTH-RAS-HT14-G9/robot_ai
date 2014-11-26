#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

//------------------------------------------------------------------------------
// Members

double _x,_y,_theta;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

visualization_msgs::Marker _robot_marker;

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
    static ros::Publisher _vis_pub;


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

    _vis_pub.publish(_robot_marker);
}

/**
  * Adapter from http://simreal.com/content/Odometry
  */
void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    static tf::TransformBroadcaster pub_tf;
    double dist_l = (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder1 / robot::prop::ticks_per_rev);
    double dist_r = (2.0*M_PI*robot::dim::wheel_radius) * (-encoders->delta_encoder2 / robot::prop::ticks_per_rev);

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

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_generator");

    ros::NodeHandle n;

    _odom.header.frame_id = "map";
    _x = _y = 0;
    _theta = 0;

    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);
    _pub_odom = n.advertise<nav_msgs::Odometry>("/pose/odometry/",10,(ros::SubscriberStatusCallback)connect_callback);

    _vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    ros::spin();

    return 0;
}
