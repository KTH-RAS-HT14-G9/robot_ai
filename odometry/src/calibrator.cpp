#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <common/robot.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <fstream>

//------------------------------------------------------------------------------
// Members

double _last_x, _last_y;

double _x,_y,_theta;
tf::Quaternion _q;
nav_msgs::Odometry _odom;

ros::Publisher _pub_odom;
ros::Publisher _pub_turn;
ros::Publisher _pub_fwd;

int _phase;

//------------------------------------------------------------------------------
// Methods

int write_result(double x, double y, double theta)
{
  using namespace std;
  stringstream filename;
  filename << "calibration_" << ros::Time::now().toNSec();

  ofstream myfile;
  myfile.open (filename.str().c_str());
  myfile << "x= " << x << "\n";
  myfile << "y= " << y << "\n";
  myfile << "theta= " << theta << "\n";
  myfile.close();
  return 0;
}

bool advance_phase() {

    if (_phase == 8) {
        ROS_ERROR("Finished round trip. Current odometry: [%lf, %lf, %lf]", _x, _y, _theta);
        write_result(_x,_y,_theta);
        ++_phase;
        return false;
    }
    else if (_phase > 8) return false;

    ++_phase;

    if (_phase%2 == 1) {

        _last_x = _x;
        _last_y = _y;

        ROS_ERROR("Commencing phase %d. Going forward for 4m.", _phase);
        std_msgs::Bool activate;
        activate.data = true;
        _pub_fwd.publish(activate);

    }
    else {
        ROS_ERROR("Commencing phase %d. Turning by 90 degrees.", _phase);

        std_msgs::Float64 angle;
        angle.data = -90.0f;
        _pub_turn.publish(angle);
    }

    return true;
}

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

double sq_dist(double x0, double y0, double x1, double y1)
{
    return (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0);
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

    const double max_d = 3.94;
    if (sq_dist(_x,_y, _last_x,_last_y) >= max_d*max_d) {
        std_msgs::Bool deactivate;
        deactivate.data = false;
        _pub_fwd.publish(deactivate);
    }
}

void connect_odometry_callback(const ros::SingleSubscriberPublisher& pub)
{
    pack_pose(_q,_odom);
    pub.publish(_odom);
}

void callback_turn_done(const std_msgs::BoolConstPtr& done)
{
    advance_phase();
}

void callback_fwd_stopped(const std_msgs::BoolConstPtr& stopped)
{
    ROS_ERROR("Current x,y = %lf, %lf. Dist = %lf", _x, _y, sqrt(sq_dist(_x,_y,_last_x,_last_y)));
    advance_phase();
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
    if (argc != 4) {
        ROS_ERROR("Missing arguments. Expected: x y theta");
        return 1;
    }

    ros::init(argc, argv, "calibrator");

    ros::NodeHandle n;

    _odom.header.frame_id = "map";

    _phase = 0;
    _x = atof(argv[1]);
    _y = atof(argv[2]);
    _theta = atof(argv[3]);

    ros::Subscriber sub_enc = n.subscribe("/arduino/encoders",10,callback_encoders);
    ros::Subscriber sub_turn_done = n.subscribe("/controller/turn/done",10,callback_turn_done);
    ros::Subscriber sub_fwd_done = n.subscribe("/controller/forward/stopped",10,callback_fwd_stopped);

    _pub_odom = n.advertise<nav_msgs::Odometry>("/pose/odometry/",10,(ros::SubscriberStatusCallback)connect_odometry_callback);
    _pub_turn = n.advertise<std_msgs::Float64>("/controller/turn/angle",10);
    _pub_fwd = n.advertise<std_msgs::Bool>("/controller/forward/active",10,true);

    ros::param::set("/controller/forward/velocity",0.2);
    advance_phase();

    ros::spin();

    return 0;
}
