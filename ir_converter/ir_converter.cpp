#include "ir_converter.h"

IRConverter::IRConverter()
    :_filter_fl(0.0)
    ,_filter_fr(0.0)
    ,_filter_bl(0.0)
    ,_filter_br(0.0)
    ,_filter_l(0.0)
    ,_filter_r(0.0)
    ,_lowpass_inertia("/perception/ir/filter_inertia",0.5)
{
    handle = ros::NodeHandle("");
    ir_subscriber = handle.subscribe("/arduino/adc", 1000, &IRConverter::IRCallback, this);
    distance_publisher = handle.advertise<ir_converter::Distance>("/perception/ir/distance", 1000);
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

void IRConverter::publishDistance()
{
    using namespace robot::ir;

    //set params
    _filter_fl.set_inertia(_lowpass_inertia());
    _filter_fr.set_inertia(_lowpass_inertia());
    _filter_bl.set_inertia(_lowpass_inertia());
    _filter_br.set_inertia(_lowpass_inertia());
    _filter_l.set_inertia(_lowpass_inertia());
    _filter_r.set_inertia(_lowpass_inertia());

    //convert to distance in meter
    double fl_side = distance(id_front_left,        ir_fl_side) + offset_front_left;
    double fr_side = distance(id_front_right,       ir_fr_side) + offset_front_right;
    double bl_side = distance(id_front_left,         ir_bl_side) + offset_rear_left;
    double br_side = distance(id_front_right,        ir_br_side) + offset_rear_right;
    double l_front = distance(id_front_long_left,   ir_l_front);
    double r_front = distance(id_front_long_right,  ir_r_front);

    //filter and publish message
    ir_converter::Distance msg;
    msg.fl_side = _filter_fl.filter(fl_side);
    msg.fr_side = _filter_fr.filter(fr_side);
    msg.bl_side = _filter_bl.filter(bl_side);
    msg.br_side = _filter_br.filter(br_side);
    msg.l_front = _filter_l.filter(l_front);
    msg.r_front = _filter_r.filter(r_front);
    distance_publisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ir_converter");
    IRConverter ir;
    ros::spin();
}
