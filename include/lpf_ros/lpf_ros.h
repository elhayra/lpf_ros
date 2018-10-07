

#ifndef LIZI_HW_LPF_H
#define LIZI_HW_LPF_H

/*
 * This class implements low pass filter
 */

#include <stdexcept>
#include <dynamic_reconfigure/server.h>
#include <lpf_ros/LpfConfig.h>
#include <std_msgs/Float64.h>

namespace lpf
{
    class Lpf
    {
    private:

        boost::shared_ptr<dynamic_reconfigure::Server<lpf_ros::LpfConfig> > server;

        double alpha_ = 0;
        double filtered_data_ = 0;

        // publish filetered and raw data
        ros::Publisher filtered_pub_, raw_pub_;

        void validateAlpha(double alpha);

        void registerDynamicParamCallback(ros::NodeHandle& nh);

        void onDynamicRequest(lpf_ros::LpfConfig &config, uint32_t level);

        void publishData(ros::NodeHandle& nh);

    public:

        Lpf();

        Lpf(const ros::NodeHandle &nh);

        Lpf(const ros::NodeHandle &nh, double alpha);

        /*
         * use this constructor only when you need the filtered
         * value to start with an initial value (instead of 0)
         */
        Lpf(const ros::NodeHandle &nh, double alpha, double init_value);

        /*
         * if empty constructor is being used,
         * calling init() is a must
         */
        void init(const ros::NodeHandle &nh, double alpha);

        void setAlpha(double alpha);

        double getAlpha() { return alpha_; }

        /*
         * pre: 0 < alpha < 1
         * post: low pass filtered data
         * param: raw_input: raw data to filter
         * this function need to be called in loop for each raw input
         */

        double filter(double raw_input);
    };

}


#endif //LIZI_HW_LPF_H
