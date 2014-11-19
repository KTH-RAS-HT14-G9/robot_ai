#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <tf/transform_broadcaster.h>

//------------------------------------------------------------------------------
// Members

double _x,_y,_theta;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

//ros::Publisher _pub_tf;
ros::Publisher _pub_odom;

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

/**
  * Adapter from http://simreal.com/content/Odometry
  */
void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    double dist_l = (2.0*M_PI*robot::dim::wheel_radius) * (encoders->delta_encoder1 / robot::prop::ticks_per_rev);
    double dist_r = (2.0*M_PI*robot::dim::wheel_radius) * (encoders->delta_encoder2 / robot::prop::ticks_per_rev);

    _theta += (dist_r - dist_l) / robot::dim::wheel_distance;

    double dist = (dist_r + dist_l) / 2.0;

    _x += dist * cos(_theta);
    _y += dist * sin(_theta);

    pack_pose(_q, _odom);
    _pub_odom.publish(_odom);
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
    _theta = M_PI/2.0;

    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);
    _pub_odom = n.advertise<nav_msgs::Odometry>("/pose/odometry/",10,(ros::SubscriberStatusCallback)connect_callback);

    ros::spin();

    return 0;
}
