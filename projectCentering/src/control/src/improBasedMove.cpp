/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <control/pid.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

double dt = 1/32.0;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
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

double measure_dist(double form_x, double form_y, double form_z, double dest_x, double dest_y, double dest_z){
    double distance_x = dest_x - form_x;
    double distance_y = dest_y - form_y;
    double distance_z = dest_z - form_z;
    double jarak_tridi = sqrt(pow(distance_x,2) + pow(distance_y,2) + pow(distance_z,2));
    return jarak_tridi;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "improBasedMove_node");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
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
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    double max_speed = 0.5;
    std::cout << "input maximum speed (0 < v < 1), 0.5 recommended : ";
    std::cin >> max_speed;
    PID mypid_x(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_y(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_z(dt, max_speed, -max_speed, 1.0, 0.6, 0);  // aman

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && (current_state.mode != "OFFBOARD" || !current_state.armed)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist vel;

    double wp[3][3] = {
        0, 0, 0,
        0, 0, 3,
        10, 0, 0
    };

    // takeoff
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
        vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);

        if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[1][0], wp[1][1], wp[1][2]) < 0.2){
            ROS_INFO("Takeoff completed");
            break;
        }
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // move to center
    bool isCount = false;
    ros::Time t0;
    double gazebo_cc_x, gazebo_cc_y, gazebo_tc_x, gazebo_tc_y;
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        gazebo_cc_x = (cc_x.data/20) * 0.25;
        gazebo_cc_y = (cc_y.data/20) * 0.25;
        gazebo_tc_x = (tc_x.data/20) * 0.25;
        gazebo_tc_y = (tc_y.data/20) * 0.25;
        if(ctr.data == false){
            vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(gazebo_cc_x, gazebo_tc_x);
            vel.linear.z = mypid_z.calculate(gazebo_cc_y, gazebo_tc_y);
        } else if(ctr.data == true){
            vel.linear.x = 0.2;
            vel.linear.y = mypid_y.calculate(gazebo_cc_x, gazebo_tc_x);
            vel.linear.z = mypid_z.calculate(gazebo_cc_y, gazebo_tc_y);
        }

        if((ctr.data == true) && (current_position.pose.position.x >= wp[2][0])){
            // vel.linear.x = mypid_x.calculate(wp[2][0], current_position.pose.position.x);
            // vel.linear.y = mypid_y.calculate(gazebo_cc_x, gazebo_tc_x);
            // vel.linear.z = mypid_z.calculate(gazebo_cc_y, gazebo_tc_y);
            // if(isCount == false){
            //     t0 = ros::Time::now();
            //     isCount == true;
            // } else if((isCount == true) && (ros::Time::now()-t0 > ros::Duration(4))){
            break;
            // }

        }
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // landing brokk
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        vel.linear.x = mypid_x.calculate(wp[2][0], current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(wp[2][1], current_position.pose.position.y);
        vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);
        
        if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[2][0], wp[2][1], wp[2][2])){
            break;
        }
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    
    // // for delay
    // ros::Time t0 = ros::Time::now();
    // while(ros::Time::now() - t0 < ros::Duration(3)){
    //     vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
    //     vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
    //     vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);
    //     cmd_pub.publish(vel);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // // to waypoint and wait
    // bool isLanding = false;
    // while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
    //     if(ctr.data == false){
    //         vel.linear.x = mypid_x.calculate(wp[2][0], current_position.pose.position.x);
    //         vel.linear.y = mypid_y.calculate(wp[2][1], current_position.pose.position.y);
    //         vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);
    //         t0 = ros::Time::now();
    //     } else if((measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[2][0], wp[2][1], wp[2][2]) < 0.2) && (ctr.data == true)){
    //         isLanding = true;
    //         break;
    //     }
    //     cmd_pub.publish(vel);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // // landing bosskuh
    // while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed && isLanding == true){
    //     vel.linear.x = mypid_x.calculate(wp[3][0], current_position.pose.position.x);
    //     vel.linear.y = mypid_y.calculate(wp[3][1], current_position.pose.position.y);
    //     vel.linear.z = mypid_z.calculate(wp[3][2], current_position.pose.position.z);
        
    //     if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[3][0], wp[3][1], wp[3][2])){
    //         break;
    //     }
    //     cmd_pub.publish(vel);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}

