#include "SerialComm.h"

SerialComm::SerialComm(){
    this->executing_task = false;
    // for(int i = 0; i < 3; i++)
    //     this->current_joint_pos[i] = 0;
}

SerialComm::SerialComm(const char* port){
    this->serial_port = open(port, O_RDWR);
    // Check for errors
    if (this->serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    this->set_tty();
    this->verbose = false;

    this->i_funcs_map["linear"] = 0.0f;
    this->i_funcs_map["expo"] = 1.0f;
    this->i_funcs_map["bounce_out"] = 2.0f;
    this->i_funcs_map["inout_cubic"] = 3.0f;
    // i_funcs_map
    this->executing_task = false;
    for(int i = 0; i < NO_OF_JOINTS; i++){
        this->current_joint_pos[i] = 0;
        this->current_joint_vel[i] = 0;
        this->current_joint_acc[i] = 0;
    }
}

void SerialComm::close_connection(){
    close(this->serial_port);
}

bool SerialComm::read_bytes(void* buffer, unsigned int n){
    unsigned int bytesRead = 0;
    char *safe_ptr;
    int result;

    safe_ptr = (char *)buffer;
    memset(buffer, '\0', n);
    while (bytesRead < n){
        result = read(this->serial_port, safe_ptr + bytesRead, n - bytesRead);
        if (result < 1 ){
            return false;
        }
        bytesRead += result;
    }
    return true;
}

char SerialComm::get_msg_type(){
    char ret;
    // this->read_bytes(buffer, BUFFER_SIZE);
    memcpy(&ret, this->inputb, sizeof(char));
    return ret;
}

void SerialComm::send_data(char msg_type){
    this->outputb[0] = msg_type;
    write(this->serial_port, this->outputb, BUFFER_SIZE);
}

void SerialComm::send_data(char msg_type, float *data, int data_length){
    this->outputb[0] = msg_type;
    this->array2buffer(&(this->outputb[1]), data, data_length);
    write(this->serial_port, this->outputb, BUFFER_SIZE);
}

void SerialComm::read_data(char *msg_type, float *data, int data_length){
    this->read_bytes(this->inputb, BUFFER_SIZE);
    *msg_type = this->get_msg_type();
    buffer2array(&(this->inputb[1]), data, data_length);
}

void SerialComm::set_tty(){
    // Reference material:
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    if(tcgetattr(this->serial_port, &(this->tty)) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    // Clear parity bit, disabling parity
    this->tty.c_cflag &= ~PARENB;
    // Clear stop field, only one stop bit used in communication
    this->tty.c_cflag &= ~CSTOPB;
    // Clear all the size bits
    this->tty.c_cflag &= ~CSIZE;
    // 8 bits per byte (most common)
    this->tty.c_cflag |= CS8;
    // Disable RTS/CTS hardware flow control
    this->tty.c_cflag &= ~CRTSCTS;
    // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    this->tty.c_cflag |= CREAD | CLOCAL;
    // Disable canonical mode (input is processed when a new line character
    // is received)
    this->tty.c_lflag &= ~ICANON;
    // Disable echo
    this->tty.c_lflag &= ~ECHO;
    // Disable erasure
    this->tty.c_lflag &= ~ECHOE;
    // Disable new-line echo
    this->tty.c_lflag &= ~ECHONL;
    // Disable interpretation of INTR, QUIT and SUSP
    this->tty.c_lflag &= ~ISIG;
    // Turn off s/w flow ctrl
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // Disable any special handling of received bytes
    this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    // Prevent special interpretation of output bytes (e.g. newline chars)
    this->tty.c_oflag &= ~OPOST;
    // Prevent conversion of newline to carriage return/line feed
    this->tty.c_oflag &= ~ONLCR;

    // Wait for up to 10s (100 deciseconds), returning as soon as any
    // data is received.
    this->tty.c_cc[VTIME] = 100;
    this->tty.c_cc[VMIN] = 0;
    cfsetispeed(&(this->tty), B115200);
    cfsetospeed(&(this->tty), B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port, TCSANOW, &(this->tty)) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

inline void SerialComm::buffer2array(char *buffer, float *array,
                                        int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(&array[i], buffer + sizeof(float)*i, sizeof(float));
}

inline void SerialComm::array2buffer(char *buffer, float *array,
                                        int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(buffer + sizeof(float)*i, &array[i], sizeof(float));
}

bool SerialComm::array_equal(float A[NO_OF_JOINTS], float B[NO_OF_JOINTS]){
    bool ret;
    ret = true;
    for(int i = 0; i < NO_OF_JOINTS; i++)
        ret = ret && (A[i] == B[i]);
    return ret;
}

void SerialComm::set_verbose(bool verbose){this->verbose = verbose;}

int SerialComm::set_angles(float target_angles[NO_OF_JOINTS]){
    return this->set_angles(target_angles, true);
}

void SerialComm::print_message(char *msgtype, float *info, int info_length){
    printf("%c ", *msgtype);
    for(int i = 0; i<info_length; i++)
        printf("%f ", info[i]);
    printf("\n");
}

int SerialComm::read_until_ok(float target_angles[NO_OF_JOINTS]){
    char msg_type;
    float arm_info[INFO_ARRAY_SIZE];
    do{        
        this->read_data(&msg_type, arm_info, INFO_ARRAY_SIZE);        
        if(msg_type == INFO_REPORT){
            for(int i = 0; i<NO_OF_JOINTS; i++){
                this->current_joint_pos[i] = arm_info[i];
                this->current_joint_vel[i] = arm_info[i + NO_OF_JOINTS];
                this->current_joint_acc[i] = arm_info[i + 2*NO_OF_JOINTS];
            }            
            if(this->verbose){
                printf("\tJoint positions:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
                        this->current_joint_pos[0], this->current_joint_pos[1],
                        this->current_joint_pos[2], this->current_joint_pos[3],
                        this->current_joint_pos[4], this->current_joint_pos[5],
                        this->current_joint_pos[6]);                        
                printf("\tJoint velocities:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
                        this->current_joint_vel[0], this->current_joint_vel[1],
                        this->current_joint_vel[2], this->current_joint_vel[3],
                        this->current_joint_vel[4], this->current_joint_vel[5],
                        this->current_joint_vel[6]);                        
                printf("\tJoint accelerations:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
                        this->current_joint_acc[0], this->current_joint_acc[1],
                        this->current_joint_acc[2], this->current_joint_acc[3],
                        this->current_joint_acc[4], this->current_joint_acc[5],
                        this->current_joint_acc[6]);
                printf("\033[3A\r");
                fflush(stdout);
            }
        }
    }while(msg_type != TASK_OK);
    if(this->verbose)
        printf("\n\n\n");
    this->executing_task = false;
    if(!this->array_equal(target_angles, this->current_joint_pos)){
        if(this->verbose){
            printf("Move error!\n");
            printf("\tRequested: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    target_angles[0], target_angles[1], target_angles[2],
                    target_angles[3], target_angles[4], target_angles[5],
                    target_angles[6]);
            printf("\tReached: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    this->current_joint_pos[0], this->current_joint_pos[1],
                    this->current_joint_pos[2], this->current_joint_pos[3],
                    this->current_joint_pos[4], this->current_joint_pos[5],
                    this->current_joint_pos[6]);
        }
        return ERROR_MOVE_FAIL;
    }
    if(this->verbose){
        printf("Move completed:\n");
        printf("\tRequested: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    target_angles[0], target_angles[1], target_angles[2],
                    target_angles[3], target_angles[4], target_angles[5],
                    target_angles[6]);
        printf("\tReached: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                this->current_joint_pos[0], this->current_joint_pos[1],
                this->current_joint_pos[2], this->current_joint_pos[3],
                this->current_joint_pos[4], this->current_joint_pos[5],
                this->current_joint_pos[6]);
    }
    return NO_ERROR;
}

bool SerialComm::is_executing_task(){return this->executing_task;}

int SerialComm::set_angles(float target_angles[NO_OF_JOINTS], bool blocking){
    char msg_type;
    float sent_angles[NO_OF_JOINTS];

    // Sending move request
    this->send_data(MOVE_REQUEST, target_angles, NO_OF_JOINTS);
    // Waiting for the start message:
    this->read_data(&msg_type, sent_angles, NO_OF_JOINTS);    
    if(msg_type != MOVE_START){
        if(this->verbose)
            printf("Board did not send start message (received '%c').\
             Aborting...\n", msg_type);
        return ERROR_MSG_NOT_START;
    }
    if(!this->array_equal(sent_angles, target_angles)){
        if(this->verbose){
            printf("Data corruption during transmission:\n");
            printf("\tSent:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    target_angles[0], target_angles[1], target_angles[2],
                    target_angles[3], target_angles[4], target_angles[5],
                    target_angles[6]);
            printf("\tReceived: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    sent_angles[0], sent_angles[1], sent_angles[2],
                    sent_angles[3], sent_angles[4], sent_angles[5],
                    sent_angles[6]);
        }
        return ERROR_DATA_CORRUPT_TRANS;
    }
    if(this->verbose)
        printf("Moving arm to positions %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                sent_angles[0], sent_angles[1], sent_angles[2],
                sent_angles[3], sent_angles[4], sent_angles[5],
                sent_angles[6]);

    this->executing_task = true;
    if(blocking)
        return this->read_until_ok(target_angles);
    this->non_blocking_task = new std::thread(&SerialComm::read_until_ok, this,
                                                target_angles);
    // this->non_blocking_task = std::thread(&SerialComm::test, this);
    return NON_BLOCKING_CALL;
}

void SerialComm::join_thread(){
    this->non_blocking_task->join();
}

int SerialComm::set_joints_max_speeds(float joint_max_speeds[NO_OF_JOINTS]){
    char msg_type;
    float sent_speeds[NO_OF_JOINTS];

    // Sending move request
    this->send_data(SPEED_REQUEST, joint_max_speeds, NO_OF_JOINTS);
    // Waiting for the ok message:
    this->read_data(&msg_type, sent_speeds, NO_OF_JOINTS);
    if(msg_type != TASK_OK){
        return ERROR_MSG_NOT_OK;
    }
    if(!this->array_equal(joint_max_speeds, sent_speeds)){
        if(this->verbose){
            printf("Data corruption during transmission:\n");
            printf("\tSent:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    joint_max_speeds[0], joint_max_speeds[1], joint_max_speeds[2],
                    joint_max_speeds[3], joint_max_speeds[4], joint_max_speeds[5],
                    joint_max_speeds[6]);
            printf("\tReceived: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    sent_speeds[0], sent_speeds[1], sent_speeds[2],
                    sent_speeds[3], sent_speeds[4], sent_speeds[5],
                    sent_speeds[6]);
        }
        return ERROR_DATA_CORRUPT_TRANS;
    }
    return NO_ERROR;
}

int SerialComm::set_joints_i_funcs(std::string *func_names){
    char msg_type;
    float float_reps[NO_OF_JOINTS], sent_float_reps[NO_OF_JOINTS];
    bool all_key_found;
    all_key_found = true;
    for(int i = 0; i < NO_OF_JOINTS; i++)
        all_key_found = all_key_found &&
            (this->i_funcs_map.find(func_names[i]) != this->i_funcs_map.end());
    if(!all_key_found)
        // Key not found
        return ERROR_NO_IFUNC_REP;
    for(int i = 0; i < NO_OF_JOINTS; i++){
        float_reps[i] = this->i_funcs_map[func_names[i]];
    }
    // Sending interp. func. representations
    this->send_data(INTERPOLATIONS_REQUEST, float_reps, NO_OF_JOINTS);
    // Waiting for the ok message:
    this->read_data(&msg_type, sent_float_reps, NO_OF_JOINTS);
    if(msg_type != TASK_OK){
        return ERROR_MSG_NOT_OK;
    }
    if(!this->array_equal(float_reps, sent_float_reps)){
        if(this->verbose){
            printf("Data corruption during transmission:\n");
            printf("\tSent:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    float_reps[0], float_reps[1], float_reps[2],
                    float_reps[3], float_reps[4], float_reps[5],
                    float_reps[6]);
            printf("\tReceived: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    sent_float_reps[0], sent_float_reps[1], sent_float_reps[2],
                    sent_float_reps[3], sent_float_reps[4], sent_float_reps[5],
                    sent_float_reps[6]);
        }
        return ERROR_DATA_CORRUPT_TRANS;
    }
    return NO_ERROR;
}

int SerialComm::update_arm_info(){
    char msg_type;    
    float arm_info[INFO_ARRAY_SIZE];         
    // Sending info request
    this->send_data(ARM_STATE_REQUEST);
    // Waiting for the ok message:    
    this->read_data(&msg_type, arm_info, INFO_ARRAY_SIZE);  
    if(msg_type != TASK_OK){
        return ERROR_MSG_NOT_OK;
    }
    for(int i = 0; i < NO_OF_JOINTS;i++){
        this->current_joint_pos[i] = arm_info[i];
        this->current_joint_vel[i] = arm_info[i + NO_OF_JOINTS];
        this->current_joint_acc[i] = arm_info[i + 2*NO_OF_JOINTS];
    }
    return NO_ERROR;
}

bool SerialComm::wait_position_set(){
    int curr_loop, max_loops;

    curr_loop = 0;
    max_loops = 1200;    
    while(this->is_executing_task()){   
        if(curr_loop > max_loops){ // Taking too long -> fail
            return false;
        }        
        usleep(0.25 * 1000000);
        curr_loop++;     
    }
    
    return true;
}