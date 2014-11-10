#ifndef IR_CONVERTER_H
#define IR_CONVERTER_H

#include <ros/ros.h>
#include <common/robot.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ir_converter/Distance.h>

class IRConverter
{
public:
    IRConverter();
    bool ok() const;
    void IRCallback(const ras_arduino_msgs::ADConverter::ConstPtr &adc);
    void publishDistance();
private:
    ros::NodeHandle handle;
    ros::Subscriber ir_subscriber;
    ros::Publisher distance_publisher;
    int ir_fl_side, ir_fr_side;
    int ir_bl_side, ir_br_side;
    int ir_l_front, ir_r_front;
};

#endif // IR_CONVERTER_H
