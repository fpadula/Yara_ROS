#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>

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
#include <YaraLowLevelComm.h>

#define PORT 1337

class Yara_FJT_Action{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        std::string action_name_;
        // create messages that are used to published feedback/result
        control_msgs::FollowJointTrajectoryFeedback feedback_;
        control_msgs::FollowJointTrajectoryResult result_;
        YaraLowLevelComm *comm;
    public:
        Yara_FJT_Action(YaraLowLevelComm *comm, std::string name) : as_(nh_, name, boost::bind(&Yara_FJT_Action::executeCB, this, _1), false),action_name_(name){
            this->comm = comm;
            as_.start();
        }

        ~Yara_FJT_Action(void){}

        void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal){
            // ROS_INFO("Received action request!");
            // helper variables
            ros::Rate r(1);
            bool success = true;
            const trajectory_msgs::JointTrajectoryPoint *cp;
            float joint_pos[6], joint_vel[6], joint_pos_t[100][6], joint_vel_t[100][6];;

            

            // // push_back the seeds for the fibonacci sequence
            // feedback_.sequence.clear();
            // feedback_.sequence.push_back(0);
            // feedback_.sequence.push_back(1);

            // publish info to the console for the user
            ROS_INFO("%s: Executing goal!", action_name_.c_str());            
            // ROS_INFO("First point: %f", goal->trajectory.points[0].positions[0]);
            // Sending whole trajectory:
            for (int i = 0; i < goal->trajectory.points.size(); i++){
                cp = &goal->trajectory.points[i];
                joint_pos_t[i][0] = cp->positions[0];
                joint_pos_t[i][1] = cp->positions[1];
                joint_pos_t[i][2] = cp->positions[2];
                joint_pos_t[i][3] = cp->positions[3];
                joint_pos_t[i][4] = cp->positions[4];
                joint_pos_t[i][5] = cp->positions[5];

                joint_vel_t[i][0] = 0.523599; // 5 degrees/s
                joint_vel_t[i][1] = 0.523599; // 5 degrees/s
                joint_vel_t[i][2] = 0.523599; // 5 degrees/s
                joint_vel_t[i][3] = 0.523599; // 5 degrees/s
                joint_vel_t[i][4] = 0.523599; // 5 degrees/s
                joint_vel_t[i][5] = 0.523599; // 5 degrees/s
            }
            this->comm->send_joint_pos_vel_t(joint_pos_t, joint_vel_t, goal->trajectory.points.size());
            // Iterating between all the trajectory points:
            for (int i = 0; i < goal->trajectory.points.size(); i++){
                // check that preempt has not been requested by the client
                if (as_.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    // set the action state to preempted
                    as_.setPreempted();
                    success = false;
                    break;
                }

                // cp = &goal->trajectory.points[i];
                // joint_pos[0] = cp->positions[0];
                // joint_pos[1] = cp->positions[1];
                // joint_pos[2] = cp->positions[2];
                // joint_pos[3] = cp->positions[3];
                // joint_pos[4] = cp->positions[4];
                // joint_pos[5] = cp->positions[5];

                // joint_vel[0] = 0.523599; // 5 degrees/s
                // joint_vel[1] = 0.523599; // 5 degrees/s
                // joint_vel[2] = 0.523599; // 5 degrees/s
                // joint_vel[3] = 0.523599; // 5 degrees/s
                // joint_vel[4] = 0.523599; // 5 degrees/s
                // joint_vel[5] = 0.523599; // 5 degrees/s
                
                success = success && this->comm->wait_position_set();
                // success = success && this->comm->send_joint_pos_vel(joint_pos, joint_vel);
                feedback_.header.stamp = ros::Time::now();
                // feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
                // publish the feedback
                as_.publishFeedback(feedback_);
                // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
                // r.sleep();
                if(!success){
                    ROS_INFO("%s: Failure!", action_name_.c_str());
                    break;
                }
            }

            if (success){
                result_.error_code = result_.SUCCESSFUL;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded(result_);
            }
        }
};

void js_publisher(ros::NodeHandle *n, int loop_freq, YaraLowLevelComm *comm){
    ros::Rate loop_rate(loop_freq);
    sensor_msgs::JointState js;
    ros::Publisher chatter_pub = n->advertise<sensor_msgs::JointState>("joint_states", 1000);

    js.name.push_back("base_to_link_1");
    js.name.push_back("link_1_to_link_2");
    js.name.push_back("link_2_to_link_3");
    js.name.push_back("link_3_to_link_4");
    js.name.push_back("link_4_to_link_5");
    js.name.push_back("link_5_to_link_6");

    for (int i = 0; i < 6; i++)
        js.position.push_back(0.0);

    while (ros::ok())
    {
        js.header.stamp = ros::Time::now();

        js.position[0] = comm->last_read_joint_positions[0];
        js.position[1] = comm->last_read_joint_positions[1];
        js.position[2] = comm->last_read_joint_positions[2];
        js.position[3] = comm->last_read_joint_positions[3];
        js.position[4] = comm->last_read_joint_positions[4];
        js.position[5] = comm->last_read_joint_positions[5];
        chatter_pub.publish(js);

        ros::spinOnce();

        loop_rate.sleep();
    }
}

int main(int argc, char **argv){
    ROS_INFO("Starting Yara low level comm...");
    YaraLowLevelComm comm("yara.local", 1337);
    ROS_INFO("Done!");

    ROS_INFO("Starting Yara Action Server...");
    ros::init(argc, argv, "yara_controller");
    ros::NodeHandle n;
    std::thread js_publisher_t(js_publisher, &n, 10, &comm);

    // ros::Subscriber sub = n.subscribe("set_yara_joint_positions", 1000, &YaraLowLevelComm::set_joint_pos_callback, &comm);

    // ros::spin();
    // js_publisher_t.join();

    // ros::init(argc, argv, "yara_controller");

    Yara_FJT_Action yfjta(&comm, "yara_controller");
    ROS_INFO("Done!");
    ros::spin();

    return 0;    
}