#include <YaraLowLevelComm.h>

YaraLowLevelComm::YaraLowLevelComm(const char *hostname, uint16_t port){
    this->hostname = hostname;
    this->port = port;
    this->send_msg = false;
    this->send_t_msg = false;
    this->position_set = false;
    this->terminate = false;
    this->connect_to_socket();
    this->communication_handler_t = std::thread(&YaraLowLevelComm::communication_handler, this);
}

YaraLowLevelComm::~YaraLowLevelComm(){
    this->terminate = true;
    this->communication_handler_t.join();
}

bool YaraLowLevelComm::send_joint_pos_vel(float *joint_pos, float *joint_vel){
    ros::Rate loop_rate(1000);
    int curr_loop, max_loops;

    curr_loop = 0;
    max_loops = 10000; // 10000 loops at 1000Hz = 10s

    memset(this->send_buffer, 0, sizeof(this->send_buffer));
    sprintf(send_buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5], joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3], joint_vel[4], joint_vel[5]);
    // std::cout<<send_buffer<<std::endl;
    this->send_msg = true;
    // this->position_set = false;
    return this->wait_position_set();
}


bool YaraLowLevelComm::wait_position_set(){
    ros::Rate loop_rate(1000);
    int curr_loop, max_loops;

    curr_loop = 0;
    max_loops = 50000;    
    while(!this->position_set){
        if(curr_loop > max_loops){ // Taking too long -> fail
            return false;
        }
        loop_rate.sleep();
        curr_loop++;
    }
    return true;
}

void YaraLowLevelComm::send_joint_pos_vel_t(float joint_pos[100][6], float joint_vel[100][6], unsigned int no_of_points){
    ros::Rate loop_rate(1000);
    int curr_loop, max_loops;
    char local_send_buffer[256];

    curr_loop = 0;
    max_loops = 10000; // 10000 loops at 1000Hz = 10s
    this->traj_size = no_of_points;
    memset(this->send_buffer, 0, sizeof(this->send_buffer));

    for(int i = 0; i < no_of_points; i++){
        memset(local_send_buffer, 0, sizeof(local_send_buffer));
        sprintf(local_send_buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", joint_pos[i][0], joint_pos[i][1], joint_pos[i][2], joint_pos[i][3], joint_pos[i][4], joint_pos[i][5], joint_vel[i][0], joint_vel[i][1], joint_vel[i][2], joint_vel[i][3], joint_vel[i][4], joint_vel[i][5]);
        strcat(this->send_buffer,local_send_buffer);
    }
    this->position_set = false;
    this->send_t_msg = true;
}

// void YaraLowLevelComm::set_joint_pos_callback(const sensor_msgs::JointState::ConstPtr& msg){
//     bool success;
//     float joint_array[6];

//     joint_array[0] = msg->position[0];
//     joint_array[1] = msg->position[1];
//     joint_array[2] = msg->position[2];
//     joint_array[3] = msg->position[3];
//     joint_array[4] = msg->position[4];
//     joint_array[5] = msg->position[5];
//     ROS_INFO("Setting pos to: [%f, %f, %f, %f, %f, %f]...", joint_array[0],joint_array[1],joint_array[2], joint_array[3], joint_array[4], joint_array[5]);
//     success = this->send_joint_pos(joint_array);
//     if(success)
//         ROS_INFO("Done!");
//     else
//         ROS_INFO("Error setting joint position!");
// }

char* YaraLowLevelComm::get_host_ip(){

    struct addrinfo hints, *res;
    int errcode;
    char *addrstr;
    void *ptr;

    addrstr = (char *)malloc(sizeof(char)*100);

    memset (&hints, 0, sizeof (hints));
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags |= AI_CANONNAME;

    errcode = getaddrinfo (this->hostname, NULL, &hints, &res);
    if (errcode != 0){
        perror ("getaddrinfo");
        return NULL;
    }

    inet_ntop (res->ai_family, res->ai_addr->sa_data, addrstr, 100);
    ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
    inet_ntop (res->ai_family, ptr, addrstr, 100);
    return addrstr;
}

int YaraLowLevelComm::connect_to_socket(){
    int valread;
    struct sockaddr_in serv_addr;
    char *addrstr;

    addrstr = this->get_host_ip();

    if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(this->port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, addrstr, &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    if (connect(this->sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    free(addrstr);
}

void YaraLowLevelComm::readXBytes(int socket, unsigned int x, void* buffer){
    int bytesRead = 0;
    int result;
    while (bytesRead < x){
        result = read(socket, buffer + bytesRead, x - bytesRead);
        if (result < 1 ){
            // Throw your error.
            printf("\nREADING ERROR!!!!\n");
        }

        bytesRead += result;
    }
}

void YaraLowLevelComm::communication_handler(){
    int valread, i;
    char read_buffer[256] = {0}, *ptr;
    unsigned int length;
    // float joint_positions[6];


    while(!this->terminate){
        // Checking to see if we need to send a message to the device:
        if(this->send_msg){
            this->send_msg = false;
            length = strlen(this->send_buffer);
            send(this->sock , (void*)&length, 4, 0);
            send(this->sock , this->send_buffer, length, 0);
        }
        if(this->send_t_msg){
            this->send_t_msg = false;
            // Sending number of waypoints:
            send(this->sock , (void*)&this->traj_size, 4, 0);
            length = strlen(this->send_buffer);
            // Sending buffer string size:
            send(this->sock , (void*)&length, 4, 0);
            // Sending buffer data:
            ROS_INFO("Sending buffer of size %u", length);
            send(this->sock , this->send_buffer, length, 0);            
        }
        // Reading message from server:
        // Setting buffer to 0:
        memset(read_buffer, 0, sizeof(read_buffer));
        // First we need to read the message length:
        this->readXBytes(this->sock, 4, (void*)&length);
        // Then we read the message if length is bigger than 0:        
        if(length > 0){
            this->readXBytes(this->sock, length, (void*)read_buffer);
            // std::cout << "[" << length << "]" << read_buffer <<std::endl;
            // Parsing message:
            // ROS_INFO("Read buffer of size %u. Message: %s", length, read_buffer);
            if(strcmp("done",read_buffer) == 0){
                // This is a done message, so done_flag must be set to true:
                this->position_set = true;
            }
            else{
                // This is a info message. Floats need to be parsed:
                i = 0;
                ptr = strtok(read_buffer, ",");
                while(ptr != NULL){
                    this->last_read_joint_positions[i] = atof(ptr);
                    i++;
                    ptr = strtok(NULL, ",");
                }
                // printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", joint_positions[0],joint_positions[1],joint_positions[2],joint_positions[3],joint_positions[4],joint_positions[5]);
            }
        }
    }
}