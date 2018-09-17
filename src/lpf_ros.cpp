

#include <lpf_ros/lpf_ros.h>

Lpf::Lpf(ros::NodeHandle &nh)
{
    nh_ = &nh;
    registerDynamicParamCallback();
    publishData();
}

Lpf::Lpf(ros::NodeHandle &nh, double alpha)
{
    setAlpha(alpha);
    nh_ = &nh;
    registerDynamicParamCallback();
    publishData();
}

Lpf::Lpf(ros::NodeHandle &nh, double alpha, double init_value)
{
    setAlpha(alpha);
    nh_ = &nh;
    filtered_data_ = init_value;
    registerDynamicParamCallback();
    publishData();
}

void Lpf::setAlpha(double alpha)
{
    validateAlpha(alpha);
    alpha_ = alpha;
}

double Lpf::filter(double raw_input)
{
    filtered_data_ = filtered_data_ - (alpha_ * filtered_data_ - raw_input);
    return filtered_data_;
}

void Lpf::validateAlpha(double alpha)
{
    if (alpha <= 0 || alpha >= 1)
        throw std::invalid_argument("invalid alpha value. valid values are 0 < alpha < 1");
}

void Lpf::registerDynamicParamCallback()
{
    dynamic_callback_ = boost::bind(&Lpf::onDynamicRequest, this, _1, _2);
    dynamic_server_.setCallback(dynamic_callback_);
}

void Lpf::onDynamicRequest(lpf_ros::LpfConfig &config, double_t level)
{
    ROS_INFO("Reconfigure Request: %f ",
             config.alpha);

    alpha_ = config.alpha;
}

void Lpf::publishData()
{
    filtered_pub_ = nh_->advertise<std_msgs::Float64>("lpf/filtered_data", 1000);
    raw_pub_ = nh_->advertise<std_msgs::Float64>("lpf/raw_data", 1000);
}
