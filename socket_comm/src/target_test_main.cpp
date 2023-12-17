#include <socket_comm/target_test.hpp>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "target_publisher");

    Target_Test target_test;
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        target_test.publish_test_data();
        loop_rate.sleep();
    }

    return 0;

}