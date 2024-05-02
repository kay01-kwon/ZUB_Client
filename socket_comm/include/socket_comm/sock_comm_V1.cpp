#include "sock_comm.hpp"

SOCK_COMM::SOCK_COMM()
{
    initialize_data();

    socket_creation();

    socket_connection();

    nh_subscriber = nh.subscribe("/target", 
                                10, 
                                &SOCK_COMM::callback_TargetData, 
                                this);

    nh_publisher = nh.advertise<actual>("actual",10);
}

void SOCK_COMM::initialize_data()
{
    recv_len = 0;

    // Init actual data
    for(int i = 0; i < 3; i++)
    {
        Actual_PAN_pos[i] = 0;
        Actual_PAN_vel[i] = 0;

        Actual_LIFT_torque[i] = 0;
        Actual_LIFT_pos[i] = 0;
        Actual_LIFT_vel[i] = 0;

        Actual_WHEEL_torque[i] = 0;
        Actual_WHEEL_vel[i] = 0;
    }

    for(int i = 0; i < SEND_BUF_LEN; i++)
        send_data[i] = 0;

    for(int i = 0; i < RECV_BUF_LEN; i++)
        recv_data[i] = 0;
}

void SOCK_COMM::socket_creation()
{

    // Create socket
    client_sock = socket(AF_INET, 
                        SOCK_STREAM, 
                        IPPROTO_TCP);

    client_sock_ptr = &client_sock;

    cout<<"socket()"<<endl;

    if(client_sock < 0)
    {
        cout<<"socket() error"<<endl;
        cout<<"Exit"<<endl;
        exit(1);
    }
}

void SOCK_COMM::socket_connection()
{
    // Set protocol
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    
    // Set destinated server address and port
    server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);
    server_addr.sin_port = htons(PORT);
    
    printf("connect()... [%s, %d]\n", IP_ADDR, PORT);

    if( connect(client_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) )
    {
        printf("connect() error[IP:%s, PORT:%d].\n",IP_ADDR, PORT);
        cout<<"Exit"<<endl;
        exit(1);
    }
    printf("Connected!\n");


}

void SOCK_COMM::callback_TargetData(const msg_pkg::targetConstPtr& target)
{
    int offset = 0;

    for(int i=0; i<NUM_PAN; i++)
    {
        target_PAN[i] = target->target_PAN[i];
        for(int j=0; j<4; j++)
        {
            send_data[j+offset] = (target_PAN[i] >> (8*j));
        }
        offset += 4;
    }

    for(int i=0; i<NUM_LIFT; i++)
    {
        target_LIFT[i] = target->target_LIFT[i];
        for(int j=0; j<4; j++)
        {
            send_data[j+offset] = (target_LIFT[i] >> (8*j));
        }
        offset += 4;
    }

    for(int i = 0; i < NUM_WHEEL; i++)
    {
        target_WHEEL[i] = target->target_WHEEL[i];
        for(int j = 0; j < 4; j++)
        {
            send_data[j + offset] = (target_WHEEL[i] >> (8*j));
        }
        offset += 4;
    }

    // for(int i=0; i<NUM_LIFT; i++)
    // {
    //     target_torque[i] = target->target_torque[i];
    //     for(int j=0; j<2; j++)
    //     {
    //         send_data[j+offset] = (target_torque[i] >> (8*j));
    //     }
    // }
}

void SOCK_COMM::publish_ActualData()
{
    send(client_sock, send_data, SEND_BUF_LEN, 0);
    recv_len = recv(client_sock, recv_data, sizeof(recv_data), 0);

    // for entire sys
    if(recv_len > 0)
        actual_data();

    // for 1 leg
    // if(recv_len > 0)
    //     one_leg_exp_data();

    nh_publisher.publish(pub_data);
}

void SOCK_COMM::actual_data()
{
    pub_data.stamp = ros::Time::now();

    // Init actual data
    for(int i = 0; i < 3; i++)
    {
        Actual_PAN_pos[i] = 0;
        Actual_PAN_vel[i] = 0;

        Actual_LIFT_torque[i] = 0;
        Actual_LIFT_pos[i] = 0;
        Actual_LIFT_vel[i] = 0;

        Actual_WHEEL_torque[i] = 0;
        Actual_WHEEL_vel[i] = 0;
    }

    // Store actual data from recv_data
    int offset = 0;
    for(int i = 0; i < 3; i++)
    {
        // PAN data
        for(int j = 0; j < 4; j++)
        {
            Actual_PAN_pos[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_PAN_pos[i] = Actual_PAN_pos[i];
        offset += 4;
        for(int j = 0; j < 4; j++)
        {
            Actual_PAN_vel[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_PAN_vel[i] = Actual_PAN_vel[i];
        offset += 4;

        // LIFT data
        for(int j = 0; j < 2; j++)
        {
            Actual_LIFT_torque[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_LIFT_torque[i] = Actual_LIFT_torque[i];
        offset += 2;

        for(int j = 0; j < 4; j++)
        {
            Actual_LIFT_pos[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_LIFT_pos[i] = Actual_LIFT_pos[i];
        offset += 4;

        for(int j = 0; j < 4; j++)
        {
            Actual_LIFT_vel[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_LIFT_vel[i] = Actual_LIFT_vel[i];
        offset += 4;

        // WHEEL data
        for(int j = 0; j < 2; j++)
        {
            Actual_WHEEL_torque[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_WHEEL_torque[i] = Actual_WHEEL_torque[i];
        offset += 2;

        for(int j = 0; j < 4; j++)
        {
            Actual_WHEEL_vel[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_WHEEL_vel[i] = Actual_WHEEL_vel[i];
        offset += 4;
    }
}