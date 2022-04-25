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
    ros::init(argc, argv, "cameraFaceDown_node");
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
    PID mypid_x(dt, max_speed, -max_speed, 0.750, 0.050, 0);
    PID mypid_y(dt, max_speed, -max_speed, 0.750, 0.050, 0);
    PID mypid_z(dt, max_speed, -max_speed, 1.000, 0.600, 0);  // aman

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1/dt);

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
        0, 0, 3.5,
        5, 8, 3.5,
    };

    // takeoff
    bool isCount = false;
    ros::Time t0;
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
        vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);

        if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[1][0], wp[1][1], wp[1][2]) < 0.2){
            if(isCount == false){
                t0 = ros::Time::now();
                isCount = true;
            }else if( (isCount == true) && (ros::Time::now()-t0 > ros::Duration(4))){
                ROS_INFO("Takeoff completed");
                break;
            }
        }

        ROS_INFO("%lf", measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[1][0], wp[1][1], wp[1][2]));
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // yok jalan
    isCount = false;
    double gazebo_cc_x, gazebo_cc_y, gazebo_tc_x, gazebo_tc_y;
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        vel.linear.x = mypid_x.calculate(wp[2][0], current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(wp[2][1], current_position.pose.position.y);
        vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);

        if( (tc_x.data != 0) && (tc_y.data != 0) ){
            break;
            // CENTERING JANGAN DITARUH SINI, MENDING DITARUH DI LOOP YANG BARU AJA, AKU KEMARIN NYOBA JADI RUSAK, KURANG TAU MASALAHNYA DIMANA
        }

        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // centering
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        gazebo_cc_x = ( (320-cc_x.data) / 20) * 0.25;
        gazebo_cc_y = ( (240-cc_y.data) / 20) * 0.25;
        gazebo_tc_x = ( (320-tc_x.data) / 20) * 0.25;
        gazebo_tc_y = ( (240-tc_y.data) / 20) * 0.25;

        vel.linear.x = -(mypid_x.calculate(gazebo_cc_y, gazebo_tc_y));
        vel.linear.y = -(mypid_y.calculate(gazebo_cc_x, gazebo_tc_x));
        vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);

        if(ctr.data == true){
            if(isCount == false){
                t0 = ros::Time::now();
                isCount = true;
            } else if((isCount == true) && (ros::Time::now()-t0 > ros::Duration(4))){
                isCount = false;
                break;
            }
        }

        // ROS_INFO("cc_x= %.3f, cc_y= %.3f, tc_x= %.3f, tc_y= %.3f", cc_x.data, cc_y.data, tc_x.data, tc_y.data);
        // ROS_INFO("gaz_cc_x= %.3f, gaz_cc_y= %.3f, gaz_tc_x= %.3f, gaz_tc_y= %.3f", gazebo_cc_x, gazebo_cc_y, gazebo_tc_x, gazebo_tc_y);
        // ROS_INFO("vel.linear.x= %.3f, vel.linear.y= %.3f", vel.linear.x, vel.linear.y);

        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // mau landing tapi bo'ong xixixie
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        gazebo_cc_x = ( (320-cc_x.data) / 20) * 0.25;
        gazebo_cc_y = ( (240-cc_y.data) / 20) * 0.25;
        gazebo_tc_x = ( (320-tc_x.data) / 20) * 0.25;
        gazebo_tc_y = ( (240-tc_y.data) / 20) * 0.25;

        vel.linear.x = -(mypid_x.calculate(gazebo_cc_y, gazebo_tc_y));
        vel.linear.y = -(mypid_y.calculate(gazebo_cc_x, gazebo_tc_x));
        vel.linear.z = mypid_z.calculate(0.3, current_position.pose.position.z);

        if( (ctr.data == true) && (current_position.pose.position.z > 0.2) && (current_position.pose.position.z < 0.4) ){
            if(isCount == false){
                t0 = ros::Time::now();
                isCount = true;
            } else if( (isCount == true) && (ros::Time::now()-t0 > ros::Duration(4)) ){
                isCount = false;
                break;
            }
        }

        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // misi selesai, back to home
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        gazebo_cc_x = ( (320-cc_x.data) / 20) * 0.25;
        gazebo_cc_y = ( (240-cc_y.data) / 20) * 0.25;
        gazebo_tc_x = ( (320-tc_x.data) / 20) * 0.25;
        gazebo_tc_y = ( (240-tc_y.data) / 20) * 0.25;

        vel.linear.x = -(mypid_x.calculate(gazebo_cc_y, gazebo_tc_y));
        vel.linear.y = -(mypid_y.calculate(gazebo_cc_x, gazebo_tc_x));
        vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);

        if( (current_position.pose.position.z > 3.4) && (current_position.pose.position.z < 3.6) ){
            vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);
        }
        if( (current_position.pose.position.x > -0.1) && (current_position.pose.position.x < 0.1) 
                    && (current_position.pose.position.y > -0.1) && (current_position.pose.position.y < 0.1) ){
            vel.linear.x = mypid_x.calculate(wp[0][0], current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(wp[0][1], current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(wp[0][2], current_position.pose.position.z);
        }

        if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[0][0], wp[0][1], wp[0][2]) < 0.2){
            break;
        }

        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}