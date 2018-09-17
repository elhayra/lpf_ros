
/* Author: Elhay Rauper*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <lpf_ros/lpf_ros.h>

/*
 * example usage of lpf_ros.h
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lpf_ros_node");
    ros::NodeHandle nh;


    while (ros::ok())
    {

        ros::spinOnce;
        ros::Rate(50).sleep();
    }
}