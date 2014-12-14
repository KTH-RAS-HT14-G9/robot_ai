//#include <imu.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <common/parameter.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

const int SIZE = 5;

void callback_activate(const std_msgs::BoolConstPtr& val);
void callback_imu(const sensor_msgs::Imu::ConstPtr& imu_reading);
float hipass(float input, float cutoff);

//sensor_msgs::Imu _imu;
bool _active;
float _accel;
float _temp_acc;

//Parameter
Parameter<double> _accel_th("/perception/imu/accel_th",2.0);
Parameter<double> _cutoff("/perception/imu/cutoff",0.5);

ros::Publisher _pub_imu;
std_msgs::Time _peak;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu");
	_active = true;
	//_peak.data = ros::Time::now();
	_accel = 0;
	_temp_acc = 0;
	ros::NodeHandle handle = ros::NodeHandle("");
	_pub_imu = handle.advertise<std_msgs::Time>("/perception/imu/peak",10);
	ros::Subscriber sub_active = handle.subscribe("/perception/imu/active",10,
						&callback_activate);
	ros::Subscriber sub_imu = handle.subscribe("/imu/data_raw",10,
						&callback_imu);

	ros::spin();
	return 0;
}

void callback_activate(const std_msgs::BoolConstPtr& val) 
{
    _active = val->data;
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& imu_reading) 
{
	_accel = imu_reading->linear_acceleration.y;
	float h_filter_out = hipass(_accel,_cutoff());
	if((h_filter_out >_accel_th()) && _active)
	{
		_peak.data = ros::Time::now();
		_pub_imu.publish(_peak);

	}	
}

float hipass(float input, float cutoff) {
	float hi_pass_output=input-(_temp_acc + cutoff*(input-_temp_acc));
	_temp_acc=hi_pass_output;
	return(hi_pass_output);
}
