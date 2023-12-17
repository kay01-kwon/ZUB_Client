#include "target_test.hpp"

Target_Test::Target_Test()
{
    pos_amp = 63504;
    vel_amp = 10;
    test_enable = false;
    nh_publisher = nh.advertise<msg_pkg::target>("target",1);
}

void Target_Test::publish_test_data()
{
    msg_pkg::target target_;

    if(test_enable == false)
    {
        time_start = ros::Time::now().toSec();
        test_enable = true;
    }
    time_pres = ros::Time::now().toSec() - time_start;

    target_.stamp = ros::Time::now();

    if(time_pres >=10)
    {
        for(int i = 0; i < 3; i++)
        {
            target_.target_pos[i] = pos_amp*sin(2*M_PI*0.2*(time_pres-10.0));
            target_.target_vel[i] = vel_amp*sin(2*M_PI*0.2*(time_pres-10.0));
        }
    }



    nh_publisher.publish(target_);

}