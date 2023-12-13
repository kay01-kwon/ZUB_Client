#ifndef SOCK_COMM_HPP
#define SOCK_COMM_HPP

#include <iostream>
#include "sock_def.hpp"
#include <stdio.h>
#include <stdlib.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include <msg_pkg/target.h>

#include <std_msgs/Int32.h>

#include <string.h>

using std::cout;
using std::endl;
using msg_pkg::target;

class SOCK_COMM{

    public:

        SOCK_COMM();

        void initialize_data();

        void socket_creation();

        void socket_connection();

        void callback_TargetData(const msg_pkg::targetConstPtr& target);

        void publish_ActualData();


    private:

        int* client_sock_ptr;
        int client_sock;
        struct sockaddr_in server_addr;

        ros::NodeHandle nh;

        ros::Publisher nh_publisher;
        ros::Subscriber nh_subscriber;


        uint8_t send_data[BUF_LEN], recv_data[BUF_LEN];

        int32_t Actual_PAN_pos[NUM_PAN];

        int16_t Actual_WHEEL_torque[NUM_WHEEL];
        int32_t Actual_WHEEL_vel[NUM_WHEEL];

        int16_t target_torque[NUM_LIFT];
        int32_t target_pos[NUM_PAN];
        int32_t target_vel[NUM_WHEEL];

    
};

#endif