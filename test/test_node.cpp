
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

TEST(Filter, LpfNoThresh)
{
    filter.resetData();
    filter.setAlpha(0.1);
    double filtered_data = 0;

    filtered_data = filter.filter(100);
    EXPECT_EQ(filtered_data, 10);
    filtered_data = filter.filter(100);
    EXPECT_EQ(filtered_data, 19);
    filtered_data = filter.filter(100);
    EXPECT_EQ(filtered_data, 27.1);
    filtered_data = filter.filter(200);
    EXPECT_EQ(filtered_data, 44.39);
}

TEST(Filter, LpfWithThresh)
{
    filter.resetData();
    filter.setAlpha(0.1);
    double filtered_data = 0;

    filtered_data = filter.filter(0.001, 0.001);
    EXPECT_EQ(filtered_data, 0);

    filter.resetData();
    filtered_data = filter.filter(0.001, 0.00001);
    EXPECT_EQ(filtered_data, 0.0001);
}

TEST(Filter, LpfNegative)
{
    filter.resetData();
    filter.setAlpha(0.1);
    double filtered_data = 0;

    filtered_data = filter.filter(-100);
    EXPECT_EQ(filtered_data, -10);
    filtered_data = filter.filter(-100);
    EXPECT_EQ(filtered_data, -19);
    filtered_data = filter.filter(-100);
    EXPECT_EQ(filtered_data, -27.1);
    filtered_data = filter.filter(-200);
    EXPECT_EQ(filtered_data, -44.39);

    filter.resetData();
    filter.setAlpha(0.1);
    filtered_data = 0;

    filtered_data = filter.filter(-0.001, 0.001);
    EXPECT_EQ(filtered_data, 0);

    filter.resetData();
    filtered_data = filter.filter(-0.001, 0.00001);
    EXPECT_EQ(filtered_data, -0.0001);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lpf_ros_node");
    ros::NodeHandle lpf_nh("some_lpf");

    filter.init(lpf_nh, 0.2);

    return RUN_ALL_TESTS();
}