#include "target_test.hpp"

Target_Test::Target_Test()
{
    pos_amp = 2048;
    test_enable = false;
    nh_publisher = nh.advertise<msg_pkg::target_dxl>("/target_dxl",1);
}

void Target_Test::publish_test_data()
{
    msg_pkg::target_dxl target_;

    if(test_enable == false)
    {
        time_start = ros::Time::now().toSec();
        test_enable = true;
    }
    time_pres = ros::Time::now().toSec() - time_start;

    target_.stamp = ros::Time::now();

    for(int i = 0; i < 3; i++)
        target_.target_dxl[i] = 2048;

    if(time_pres >=10)
    {
        for(int i = 0; i < 3; i++)
        {
            target_.target_dxl[i] = pos_amp*sin(2*M_PI*0.5*(time_pres-1.0)) + 2048;
        }
    }



    nh_publisher.publish(target_);

}