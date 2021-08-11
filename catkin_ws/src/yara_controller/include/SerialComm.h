#ifndef _SERIALCOMM_H_
#define _SERIALCOMM_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdint>

#include <string>
#include <iostream>

#include <map>
#include <thread>

// #include "ikfast.h"
#include <cmath>
/*
Buffer size in bytes. We need 1 byte for msg type, 7x4=28 bytes for position
information, 7x4=28 bytes for speed information and 7x4=28 bytes for 
acceleration information. So we need 1 + 28 + 28 + 28 = 85 bytes for our buffer
*/
#define BUFFER_SIZE 85
#define INFO_ARRAY_SIZE 21
/******** Message types definitions ********/
#define INFO_REPORT 'r'
#define MOVE_START 's'
#define TASK_OK 'o'
#define TASK_ERROR 'e'
#define MOVE_REQUEST 't'
#define SPEED_REQUEST 'p'
#define INTERPOLATIONS_REQUEST 'i'
#define ARM_STATE_REQUEST 'a'
/******** Function return codes ********/
#define NO_ERROR 0;
#define ERROR_MSG_NOT_OK 1;
#define ERROR_DATA_CORRUPT_TRANS 2;
#define ERROR_MSG_NOT_START 3;
#define ERROR_NO_IFUNC_REP 4;
#define ERROR_MOVE_FAIL 5;
#define NON_BLOCKING_CALL 6;
/******** Robot variables ********/
#define NO_OF_JOINTS 7
/*********************************/


class SerialComm{
    private:
        int serial_port, buffer_size;
        struct termios tty;
        char inputb[BUFFER_SIZE], outputb[BUFFER_SIZE];
        bool verbose;
        volatile bool executing_task;
        std::map<std::string, float> i_funcs_map;
        

        void set_tty();
        
        bool read_bytes(void*, unsigned int);
        inline void buffer2array(char *, float *, int);
        inline void array2buffer(char *, float *, int);
        char get_msg_type();        
        void send_data(char);
        void send_data(char, float *, int);
        void read_data(char *, float *, int);
        bool array_equal(float [NO_OF_JOINTS], float [NO_OF_JOINTS]);

        int read_until_ok(float [NO_OF_JOINTS]);
        std::thread *non_blocking_task;
        void print_message(char *, float *, int);

        bool is_configuration_valid(double *);
        double get_solution_distance(double *, double *);
        double trig_dist(double , double);
    public:
        float current_joint_pos[NO_OF_JOINTS];
        float current_joint_vel[NO_OF_JOINTS];
        float current_joint_acc[NO_OF_JOINTS];
        SerialComm();
        SerialComm(const char *);
        // ~Controller();
        int update_arm_info();
        int set_angles(float [NO_OF_JOINTS]);
        int set_angles(float [NO_OF_JOINTS], bool);
        int set_joints_max_speeds(float [NO_OF_JOINTS]);
        int set_joints_i_funcs(std::string *);
        void set_verbose(bool);
        void close_connection();
        bool is_executing_task();
        bool wait_position_set();
        void join_thread();        
};

#endif