#include "ir_converter.h"

IRConverter::IRConverter()
{
    handle = ros::NodeHandle("");
    ir_subscriber = handle.subscribe("/arduino/adc", 1000, &IRConverter::IRCallback, this);
    distance_publisher = handle.advertise<ir_converter::Distance>("/robot_ai/distance", 1000);

}

void IRConverter::IRCallback(const ras_arduino_msgs::ADConverter::ConstPtr &adc)
{
    ir_fl_side = adc->ch1;
    ir_fr_side = adc->ch2;
    ir_bl_side = adc->ch3;
    ir_br_side = adc->ch4;
    ir_l_front = adc->ch7;
    ir_r_front = adc->ch8;
    publishDistance();
}

bool IRConverter::ok() const {
    return handle.ok();
}

void IRConverter::publishDistance()
{
    double fl_side = robot::ir::distance(robot::ir::id_front_left, ir_fl_side);
    double fr_side = robot::ir::distance(robot::ir::id_front_right, ir_fr_side);
    double bl_side = robot::ir::distance(robot::ir::id_rear_left, ir_bl_side);
    double br_side = robot::ir::distance(robot::ir::id_rear_right, ir_br_side);
    double l_front = robot::ir::distance(robot::ir::id_front_long_left, ir_l_front);
    double r_front = robot::ir::distance(robot::ir::id_front_long_right, ir_r_front);
    ir_converter::Distance msg;
    msg.fl_side = fl_side;
    msg.fr_side = fr_side;
    msg.bl_side = bl_side;
    msg.br_side = br_side;
    msg.l_front = l_front;
    msg.r_front = r_front;
    distance_publisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ir_converter");
    IRConverter ir;
    ros::Rate loop_rate(robot::prop::encoder_publish_frequency); // change to something else?

    while(ir.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
