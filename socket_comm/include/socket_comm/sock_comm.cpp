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
    send(client_sock, send_data, SEND_BUF_LEN, 0);

    // memset(&recv_data, 0, sizeof(recv_data));

    recv_len = recv(client_sock, recv_data, sizeof(recv_data), 0);

    if(recv_len > 0)
        actual_data();

    nh_publisher.publish(pub_data);
}

void SOCK_COMM::actual_data()
{
    int offset = 0;
    // 2. Init pan position data
    for(int i = 0; i < NUM_PAN; i++)
    {
        Actual_PAN_pos[i] = 0;
    }

    // 3. Init wheel torque and velocity data
    for(int i = 0; i < NUM_WHEEL; i++)
    {
        Actual_WHEEL_torque[i] = 0;
        Actual_WHEEL_vel[i] = 0;
    }

    for(int i = 0; i < NUM_PAN; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            Actual_PAN_pos[i] |= recv_data[j + offset]<<(8*j);
        }
        pub_data.act_PAN_pos[i] = Actual_PAN_pos[i];
        //cout<<"PAN_POS[ "<<i<<" ]:  "<<Actual_PAN_pos[i]<<"\t";
        offset += 4;
    }

    for(int i = 0; i < NUM_WHEEL; i++)
    {
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