
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
    ros::NodeHandle lpf_nh(nh, "some_lpf");

    lpf::Lpf filter(lpf_nh, 0.2);


    while (ros::ok())
    {
        // generate fake noise
        int rand_num = 100*sin(0.5*ros::Time::now().toSec()) +  5*sin(50*ros::Time::now().toSec());

        filter.filter(rand_num);
        ros::spinOnce;
        ros::Rate(50).sleep();
    }
}