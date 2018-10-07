
/* Author: Elhay Rauper*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <lpf_ros/lpf_ros.h>
#include <gtest/gtest.h>

lpf::Lpf filter;

TEST(ValidValues, AlphaUpdate)
{
    // test valid values
    try
    {
        filter.setAlpha(1);
        filter.setAlpha(0);
        filter.setAlpha(0.5);
    }
    catch (std::invalid_argument exp)
    {
        ADD_FAILURE() << "Invalid argumant exception was thrown for valid values";
    }
    catch (...)
    {
        ADD_FAILURE() << "No exception suppose to be thrown for valid values";
    }
}

TEST(InvalidValues, AlphaUpdate)
{
    // test invalid values
    try
    {
        filter.setAlpha(1.2);
        filter.setAlpha(-0.2);
    }
    catch (std::invalid_argument exp)
    {
        SUCCEED();
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception";
    }
}

TEST(ValueUpdate, AlphaUpdate)
{
    filter.setAlpha(0.5);
    ASSERT_EQ(0.5, filter.getAlpha());
}

TEST(Filter, Lpf)
{
    filter.setAlpha(0.1);
    double alpha;

    alpha = filter.filter(100);
    EXPECT_EQ(alpha, 10);
    alpha = filter.filter(100);
    EXPECT_EQ(alpha, 19);
    alpha = filter.filter(100);
    EXPECT_EQ(alpha, 27.1);
    alpha = filter.filter(200);
    EXPECT_EQ(alpha, 44.39);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lpf_ros_node");
    ros::NodeHandle lpf_nh("some_lpf");

    filter.init(lpf_nh, 0.2);

    return RUN_ALL_TESTS();
}