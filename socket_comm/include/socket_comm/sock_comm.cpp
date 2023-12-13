#include "sock_comm.hpp"

SOCK_COMM::SOCK_COMM()
{
    initialize_data();

    socket_creation();

    socket_connection();

    nh_subscriber = nh.subscribe("/target", 
                                1, 
                                &SOCK_COMM::callback_TargetData, 
                                this);
    
    nh_publisher = nh.advertise<std_msgs::Int32>("actual",1);

}

void SOCK_COMM::initialize_data()
{
    // 1. Initialize actual data for LIFT motors

    // 2. Initialize actual data for PAN motors
    for(int i = 0; i < NUM_PAN; i++)
        Actual_PAN_pos[i] = 0;

    // 3. Initialize actual data for WHEEL motors
    for(int i = 0; i < NUM_WHEEL; i++)
    {
        Actual_WHEEL_torque[i] = 0;
        Actual_WHEEL_vel[i] = 0;
    }

    for(int i = 0; i < BUF_LEN; i++)
    {
        send_data[i] = 0;
        recv_data[i] = 0;
    }
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

    for(int i = 0; i < NUM_LIFT; i++)
        target_torque[i] = target->target_torque[i];

    for(int i = 0; i < NUM_PAN; i++)
    {
        target_pos[i] = target->target_pos[i];
        
        for(int j = 0; j < 4; j++)
        {
            send_data[j + offset] = (target_pos[i] >> (8*j));
        }
        offset += 4;
    }

    cout<<"target_pos[0] : "<<target_pos[0]<<endl;


    for(int i = 0; i < NUM_WHEEL; i++)
    {
        target_vel[i] = target->target_vel[i];

        for(int j = 0; j < 4; j++)
        {
            send_data[j + offset] = (target_vel[i] >> (8*j));
        }
        offset += 4;
    }

}

void SOCK_COMM::publish_ActualData()
{
    send(client_sock, send_data, BUF_LEN, 0);
    std_msgs::Int32 test;
    Actual_PAN_pos[0] = 1;

    test.data = Actual_PAN_pos[0];

    nh_publisher.publish(test);
}