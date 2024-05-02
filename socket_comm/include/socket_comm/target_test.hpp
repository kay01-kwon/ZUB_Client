#ifndef TARGET_TEST_HPP
#define TARGET_TEST_HPP

#include <iostream>
#include <ros/ros.h>
#include <msg_pkg/target_dxl.h>

class Target_Test{

    public:

        Target_Test();

        void publish_test_data();

    private:

        ros::NodeHandle nh;

        ros::Publisher nh_publisher;

        double time_start;
        double time_pres;
        double time_run;

        int32_t target_pos[3];

        int32_t pos_amp;

        bool test_enable;

};

#endif