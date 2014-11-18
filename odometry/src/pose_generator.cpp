#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <tf/transform_broadcaster.h>

const double _rad_per_tick = (robot::dim::wheel_radius*2.0) / (robot::dim::wheel_distance) / 2.0 / (180.0 * M_PI);
const double _dist_per_tick = M_PI * (robot::dim::wheel_radius*2.0) / robot::prop::ticks_per_rev;

double _x,_y,_theta;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

//ros::Publisher _pub_tf;
ros::Publisher _pub_odom;

void pack_pose(tf::Quaternion& q, nav_msgs::Odometry& odom)
{
    q.setRPY(0, 0, _theta);

    odom.pose.pose.position.x = _x;
    odom.pose.pose.position.y = _y;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
}

void callback_encoders(const ras_arduino_msgs::EncodersConstPtr& encoders)
{
    _odom.header.stamp = ros::Time::now();

    _theta += (encoders->delta_encoder1 - encoders->delta_encoder2)*_rad_per_tick;
    double dDist = (encoders->delta_encoder1 + encoders->delta_encoder2) / 2.0 * _dist_per_tick;

    _x += dDist * cos(_theta);
    _y += dDist * sin(_theta);

    pack_pose(_q, _odom);

    _pub_odom.publish(_odom);
}

void connect_callback(const ros::SingleSubscriberPublisher& pub)
{
    pack_pose(_q,_odom);
    pub.publish(_odom);
}


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
