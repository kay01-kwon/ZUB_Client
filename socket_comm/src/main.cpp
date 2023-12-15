#include <socket_comm/sock_comm.hpp>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "main_test");

    SOCK_COMM sock_comm;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();

        sock_comm.publish_ActualData();

        loop_rate.sleep();
    }

    cout<<"Exit"<<endl;

    return 0;

}