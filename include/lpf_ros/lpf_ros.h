

#ifndef LIZI_HW_LPF_H
#define LIZI_HW_LPF_H

/*
 * This class implements low pass filter
 */

#include <stdexcept>
#include <dynamic_reconfigure/server.h>
#include <lpf_ros/LpfConfig.h>
#include <std_msgs/Float64.h>

class Lpf
{
private:

    double alpha_ = 0;
    double filtered_data_ = 0;

    ros::NodeHandle* nh_;
    // publish filetered and raw data
    ros::Publisher filtered_pub_, raw_pub_;

    dynamic_reconfigure::Server<lpf_ros::LpfConfig> dynamic_server_;
    dynamic_reconfigure::Server<lpf_ros::LpfConfig>::CallbackType dynamic_callback_;

public:

    Lpf(ros::NodeHandle &nh);

    Lpf(ros::NodeHandle &nh, double alpha);

    /*
     * use this constructor only when you need the filtered
     * value to start with an initial value (instead of 0)
     */
    Lpf(ros::NodeHandle &nh, double alpha, double init_value);

    void setAlpha(double alpha);

    /*
     * pre: 0 < alpha < 1
     * post: low pass filtered data
     * param: raw_input: raw data to filter
     * this function need to be called in loop for each raw input
     */

    double filter(double raw_input);

    void validateAlpha(double alpha);

    void registerDynamicParamCallback();

    void onDynamicRequest(lpf_ros::LpfConfig &config, double_t level);

    void publishData();
};

#endif //LIZI_HW_LPF_H
