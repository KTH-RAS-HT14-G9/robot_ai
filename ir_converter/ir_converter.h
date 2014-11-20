#ifndef IR_CONVERTER_H
#define IR_CONVERTER_H

#include <ros/ros.h>
#include <common/robot.h>
#include <common/lowpass_filter.h>
#include <common/parameter.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ir_converter/Distance.h>

class IRConverter
{
public:
    IRConverter();
    void IRCallback(const ras_arduino_msgs::ADConverter::ConstPtr &adc);
    void publishDistance();
private:
    ros::NodeHandle handle;
    ros::Subscriber ir_subscriber;
    ros::Publisher distance_publisher;
    int ir_fl_side, ir_fr_side;
    int ir_bl_side, ir_br_side;
    int ir_l_front, ir_r_front;

    common::LowPassFilter _filter_fl, _filter_fr;
    common::LowPassFilter _filter_bl, _filter_br;
    common::LowPassFilter _filter_l, _filter_r;

    Parameter<double> _lowpass_inertia;

};

#endif // IR_CONVERTER_H
