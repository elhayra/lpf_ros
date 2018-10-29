

#include <lpf_ros/lpf_ros.h>

namespace lpf
{
    Lpf::Lpf() {}

    Lpf::Lpf(const ros::NodeHandle &nh)
    {
        ros::NodeHandle node(nh);
        registerDynamicParamCallback(node);
        publishData(node);
    }

    Lpf::Lpf(const ros::NodeHandle &nh, double alpha)
    {
        init(nh, alpha);
    }

    Lpf::Lpf(const ros::NodeHandle &nh, double alpha, double init_value)
    {
        init(nh, alpha);
        filtered_data_ = init_value;
    }

    void Lpf::setAlpha(double alpha)
    {
        validateAlpha(alpha);
        alpha_ = alpha;
    }

    void Lpf::init(const ros::NodeHandle &nh, double alpha)
    {
        ros::NodeHandle node(nh);
        setAlpha(alpha);
        registerDynamicParamCallback(node);
        publishData(node);
    }

    double Lpf::filter(double raw_input)
    {
        std_msgs::Float64 raw_msg;
        raw_msg.data = raw_input;
        raw_pub_.publish(raw_msg);

        filtered_data_ = filtered_data_ - (alpha_ * (filtered_data_ - raw_input));

        std_msgs::Float64 filtered_msg;
        filtered_msg.data = filtered_data_;
        filtered_pub_.publish(filtered_msg);

        return filtered_data_;
    }

    double Lpf::filter(double raw_input, double zero_thresh)
    {
        if (zero_thresh <= 0 || zero_thresh >= 1)
            throw std::invalid_argument("invalid zero_thresh value. "
                                        "valid values are 0 < zero_thresh < 1");

        filter(raw_input);
        if (fabs(filtered_data_) < zero_thresh)
            resetData();
        return filtered_data_;
    }

    void Lpf::validateAlpha(double alpha)
    {
        if (alpha < 0 || alpha > 1)
            throw std::invalid_argument("invalid alpha value. valid values are 0 <= alpha <= 1");
    }

    void Lpf::registerDynamicParamCallback(ros::NodeHandle& nh)
    {
        server = boost::make_shared <dynamic_reconfigure::Server<lpf_ros::LpfConfig> > (nh);
        dynamic_reconfigure::Server<lpf_ros::LpfConfig>::CallbackType param_config_callback_ = boost::bind(&Lpf::onDynamicRequest, this, _1, _2);
        server->setCallback(param_config_callback_);
    }

    void Lpf::onDynamicRequest(lpf_ros::LpfConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure Request: %f ", config.alpha);
        alpha_ = config.alpha;
    }

    void Lpf::publishData(ros::NodeHandle& nh)
    {
        filtered_pub_ = nh.advertise<std_msgs::Float64>("lpf/filtered_data", 1000);
        raw_pub_ = nh.advertise<std_msgs::Float64>("lpf/raw_data", 1000);
    }
}
