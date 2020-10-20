#ifndef YARA_LOW_LEVEL_COMM_H
#define YARA_LOW_LEVEL_COMM_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <iostream>
#include <algorithm>
#include <thread>

#include <sstream>

class YaraLowLevelComm{
    private:
        bool position_set, send_msg, send_t_msg, terminate;
        int sock;
        unsigned int traj_size;
        const char *hostname;
        uint16_t port;
        char send_buffer[25600];
        std::thread communication_handler_t;

        char *get_host_ip();

        int connect_to_socket();
        void readXBytes(int socket, unsigned int x, void* buffer);

    public:
        float last_read_joint_positions[6];

        YaraLowLevelComm(const char *hostname, uint16_t port);
        ~YaraLowLevelComm();
        void communication_handler();
        // void set_joint_pos_callback(const sensor_msgs::JointState::ConstPtr& msg);
        bool send_joint_pos_vel(float *joint_pos, float *joint_vel);
        void send_joint_pos_vel_t(float joint_pos[100][6], float joint_vel[100][6], unsigned int no_of_points);
        bool wait_position_set();

};
#endif