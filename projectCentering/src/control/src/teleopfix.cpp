/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "control/pid.hpp"

//PID set up
double dt = 1/32.0;
// double max_val = 0.5;
// double min_val = -0.5;
// double Kp = 0.45;
// double Ki = 0.005;
// double Kd = 0.0007;
// PID mypid_x(dt, max_val, min_val, Kp, Kd, Ki);
// PID mypid_y(dt, max_val, min_val, Kp, Kd, Ki);
// PID mypid_z(dt, max_val, min_val, Kp+0.18, Kd, Ki);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

geometry_msgs::Twist curTeleVel;
void getVel(const geometry_msgs::Twist::ConstPtr &msg){
    curTeleVel = *msg;
}

std_msgs::Bool is_requesting_auto_land;
void auto_land_cb(const std_msgs::Bool::ConstPtr& msg){
    is_requesting_auto_land = *msg;
}

std_msgs::Float32 cc_x;
void cc_x_callback(const std_msgs::Float32::ConstPtr& msg){
    cc_x = * msg;
}
std_msgs::Float32 cc_y;
void cc_y_callback(const std_msgs::Float32::ConstPtr& msg){
    cc_y = * msg;
}
std_msgs::Float32 tc_x;
void tc_x_callback(const std_msgs::Float32::ConstPtr& msg){
    tc_x= * msg;
}
std_msgs::Float32 tc_y;
void tc_y_callback(const std_msgs::Float32::ConstPtr& msg){
    tc_y = * msg;
}
std_msgs::Bool ctr;
void ctr_callback(const std_msgs::Bool::ConstPtr& msg){
    ctr = *msg;
}

float jarak_titik(float tujuan_x, float tujuan_y, float tujuan_z, float asal_x, float asal_y, float asal_z){
    float jarak_x = tujuan_x - asal_x;
    float jarak_y = tujuan_y - asal_y;
    float jarak_z = tujuan_z - asal_z;
    float jarak_total = sqrt(pow(jarak_x, 2) + pow(jarak_y, 2) + pow(jarak_z, 2));
    return jarak_total;
}

bool velocity_empty(geometry_msgs::Twist &cmd_tel){
    if(cmd_tel.angular.x == 0 &&
       cmd_tel.angular.y == 0 &&
       cmd_tel.angular.z == 0 &&
       cmd_tel.linear.x == 0 &&
       cmd_tel.linear.y == 0 &&
       cmd_tel.linear.z == 0){
           return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> //Publish velocity
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped> //Subscribe position
            ("mavros/local_position/pose", 10, callback);
    ros::Subscriber teleop_vel_sub = nh.subscribe<geometry_msgs::Twist> //subs teleop vel
            ("/cmd_vel", 10, getVel);
    ros::Subscriber auto_land_sub = nh.subscribe<std_msgs::Bool> //subscribe trigger for autoland
            ("/auto_land_cmd", 10, auto_land_cb);
    ros::Subscriber cc_x_sub = nh.subscribe<std_msgs::Float32>
            ("cc/x", 10, cc_x_callback);
    ros::Subscriber cc_y_sub = nh.subscribe<std_msgs::Float32>
            ("cc/y", 10, cc_y_callback);
    ros::Subscriber tc_x_sub = nh.subscribe<std_msgs::Float32>
            ("tc/x", 10, tc_x_callback);
    ros::Subscriber tc_y_sub = nh.subscribe<std_msgs::Float32>
            ("tc/y", 10, tc_y_callback);
    ros::Subscriber ctr_sub = nh.subscribe<std_msgs::Bool>
            ("ctr", 10, ctr_callback);


    //for wp
    // ros::Publisher wp_x = nh.advertise<std_msgs::Float32>
    //         ("wp/x", 10);
    // ros::Publisher wp_y = nh.advertise<std_msgs::Float32>
    //         ("wp/y", 10);
    // ros::Publisher wp_z = nh.advertise<std_msgs::Float32>
    //         ("wp/z", 10);

    double max_speed = 0.5;
    printf("Input maximum speed sir (0 < v < 1) recommended 0.5: ");
    scanf("%lf", &max_speed);           
    PID mypid_z_takeoff(dt, max_speed, -max_speed, 1.000, 0.355, 0.01500);
    PID mypid_x(dt, max_speed, -max_speed, 0.750, 0.500, 0.00000); // GANTI!!! sesuai rate class!!!
    PID mypid_y(dt, max_speed, -max_speed, 0.750, 0.500, 0.00000);
    PID mypid_z(dt, max_speed, -max_speed, 0.350, 0.015, 0.00000);

    double target_height = 1.5;//target height in m
    printf("How about target height for takeoff? (recommended 2m): ");
    scanf("%lf", &target_height);
    
    printf("Drone will takeoff with this setting\n");
    printf("max-speed: %lf\n", max_speed);
    printf("target height: %lf\n", target_height);
    
    char confirmation;
    printf("Ready? (Enter random character or c for cancel)\n");
    scanf("\n%c", &confirmation);
    printf("You just typed: %c\n", confirmation);
    if(confirmation == 'c' || confirmation == 'C'){
        printf("Canceling flight\n");
        return 0;
    }

    printf("STARTING MISSION\n");
    
    ros::Rate rate(1/dt);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0;

    //for last commanded velocity takeoff
    // geometry_msgs::PoseStamped last_position;

    //velocity for takeoff
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    
    //Loop until drone is on offboard mode and is armed
    while(ros::ok() && (current_state.mode != "OFFBOARD" || !current_state.armed)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                printf("result arm response: %d\nresult arm success: %d\n", arm_cmd.response.result, arm_cmd.response.success);
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                // std::cout << "this is the result: " << arm_cmd.response.result << std::endl;
                // std::cout << "this is the succes: " << arm_cmd.response.success << std::endl;
            }
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    int wp_next = 0;
    bool takeoff = false;
    //Takeoff loop
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed && !takeoff){
        //calculate z velocity (target, current)
        vel.linear.z = mypid_z_takeoff.calculate(target_height, current_position.pose.position.z);
        //set other vel to 0
        vel.linear.y = 0;
        vel.linear.x = 0;

        cmd_pub.publish(vel);
        ROS_INFO("Takeoff | z < %fm | z: %f", target_height,current_position.pose.position.z);
        if(current_position.pose.position.z >= target_height){
            takeoff = true;
            ROS_INFO("Succesfully reach >%fm | CHANGE TO TELEOP COMMAND", target_height);
        }
        // last_position = current_position;
        //important so that ros process continue running
        ros::spinOnce();
        rate.sleep();
    }
    
    if(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed) ROS_INFO("START TELEOP MODE");
    //Teleop mode
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        // ROS_INFO("cc_x= %f  |  cc_y= %f\n", cc_x.data, cc_y.data);
        // ROS_INFO("tc_x= %f  |  tc_y= %f\n", tc_x.data, tc_y.data);
        // ROS_INFO("is centered= %i", ctr.data);

        //check if there is no teleop input
        // if(velocity_empty(curTeleVel))
        // {   
        //     ROS_INFO("LAST: x = %f, y = %f, z = %f", last_position.pose.position.x, last_position.pose.position.y, last_position.pose.position.z);
        //     //pertahankan posisi
        //     ROS_INFO("No command received, trying to stay in the last position recorded");
        //     vel.linear.x = mypid_x.calculate(last_position.pose.position.x, current_position.pose.position.x);
        //     vel.linear.y = mypid_y.calculate(last_position.pose.position.y, current_position.pose.position.y);
        //     vel.linear.z = mypid_z.calculate(last_position.pose.position.z, current_position.pose.position.z);
        //     cmd_pub.publish(vel);
        // }
        // else{
        cmd_pub.publish(curTeleVel);
            //update last position
            // last_position = current_position;
        // }

        if (current_position.pose.position.z < 0.5 && is_requesting_auto_land.data){
            ROS_INFO("REQUESTING AUTO LAND");
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            ROS_INFO("Landing...");
        }
        ROS_INFO("x = %f, y = %f, z = %f", current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z);
        ros::spinOnce();
        rate.sleep();
    }
    
    if(current_state.mode == "AUTO.LAND") ROS_INFO("Successfully changed to auto land");
    else
    {
        ROS_INFO("MAY DAY!, MAYDAY!\ncurrent state mode is not autoland");
        std::cout <<"current state is: " << current_state.mode << std:: endl;
    }
    return 0;
}
