#ifndef TARGET_TEST_HPP
#define TARGET_TEST_HPP

#include <iostream>
#include <ros/ros.h>
#include <msg_pkg/target.h>

#include "sock_def.hpp"

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

        int16_t target_torque[NUM_LIFT];
        int32_t target_pos[NUM_PAN];
        int32_t target_vel[NUM_WHEEL];

        int32_t pos_amp;
        int32_t vel_amp;

        bool test_enable;

};

#endif