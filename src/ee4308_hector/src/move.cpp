#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <signal.h>
#include "common.hpp"
#include <opencv2/core/matx.hpp>
#include <nav_msgs/Odometry.h> 

#define NaN std::numeric_limits<double>::quiet_NaN()

ros::ServiceClient en_mtrs;
void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv); 
}

double target_x = NaN, target_y = NaN, target_z = NaN;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
}

double x = NaN, y = NaN, z = NaN, a = NaN;
void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}

//for pid tuning
nav_msgs::Odometry msg_true;
void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

bool rotate = false;
void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    if (!enable_move)
        return 0;
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    bool pid_tuning_z, pid_tuning_xy;
    if (!nh.param("pid_tuning_z", pid_tuning_z, false))
        ROS_WARN("HMOTION: Param pid_tuning_z not found, set to false");
    if (!nh.param("pid_tuning_xy", pid_tuning_xy, false))
        ROS_WARN("HMOTION: Param pid_tuning_xy not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double Kp_z;
    if (!nh.param("Kp_z", Kp_z, 1.0))
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    double Ki_z;
    if (!nh.param("Ki_z", Ki_z, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_z;
    if (!nh.param("Kd_z", Kd_z, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double yaw_rate;
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    double max_z_vel;
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");
    double initial_x, initial_y, initial_z;
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");
    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = true;
    if (en_mtrs.call(en_mtrs_srv))
        ROS_INFO(" HMOVE : Motors enabled!");
    else
        ROS_WARN(" HMOVE : Cannot enable motors!");
    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue); // for PID tuning
    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(target_x) || std::isnan(x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce(); // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z, cmd_lin_vel_a;
    double dt;
    double prev_time = ros::Time::now().toSec();

    cv::Vec3d lin_integral = {0,0,0};
    cv::Vec3d e_prev = {0,0,0};
    cv::Vec3d lin_vel = {0,0,0};

    // //variables used for pid tuning
    double rise_time_start = -1;
    double rise_time_end = -1;
    double max_overshoot = 0;

    double PID_TESTING_SETPOINT = 2;
    bool height_reached = false;
    
    
    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;
        cmd_lin_vel_a = 0;
        if (rotate){
            cmd_lin_vel_a = yaw_rate;
        }
        cv::Matx33d rotation = { cos(a), sin(a), 0,
                                -sin(a), cos(a), 0,
                                 0     , 0     , 1 };

        if (pid_tuning_z){
            target_x = initial_x;
            target_y = initial_y;
            target_z = initial_z + PID_TESTING_SETPOINT;
        } else if (pid_tuning_xy){
            if (abs(z - PID_TESTING_SETPOINT) > 0.2 && !height_reached){
                target_x = initial_x;
                target_y = initial_y;
                target_z = PID_TESTING_SETPOINT;
            } else {
                height_reached = true;
                target_x = initial_x + PID_TESTING_SETPOINT;
                target_y = initial_y;
                target_z = PID_TESTING_SETPOINT;
            }
                
        }

        double e_x = target_x - x;
        double e_y = target_y - y;
        double e_z = target_z - z;

        cv::Vec3d lin_e = {e_x, e_y, e_z};
        cv::Vec3d lin_derivative = (lin_e - e_prev) / dt;
        lin_integral += lin_e * dt;
        e_prev = lin_e;

        cv::Vec3d Kp = {Kp_lin, Kp_lin, Kp_z};
        cv::Vec3d Ki = {Ki_lin, Ki_lin, Ki_z};
        cv::Vec3d Kd = {Kd_lin, Kd_lin, Kd_z};

        lin_vel = (Kp.mul(lin_e)) + (Ki.mul(lin_integral)) + (Kd.mul(lin_derivative));
        lin_vel = rotation * lin_vel;
        cmd_lin_vel_x = lin_vel(0);
        cmd_lin_vel_y = lin_vel(1);
        cmd_lin_vel_z = lin_vel(2);

        // publish speeds
        msg_cmd.linear.x = std::min(cmd_lin_vel_x, max_lin_vel);
        msg_cmd.linear.y = std::min(cmd_lin_vel_y, max_lin_vel);
        msg_cmd.linear.z = std::min(cmd_lin_vel_z, max_z_vel);
        msg_cmd.angular.z = cmd_lin_vel_a;
        pub_cmd.publish(msg_cmd);

        //pid tuning code for z
        if (pid_tuning_z){
            if (z > initial_z + (PID_TESTING_SETPOINT * 0.1) ) { //include the starting offset of 0.178
                if (rise_time_start == -1){
                    rise_time_start = ros::Time::now().toSec();
                    ROS_INFO("Start recording Z axis rise time");
                }
            }
            if (z >= initial_z + (PID_TESTING_SETPOINT * 0.9)) { 
                if (rise_time_end == -1){
                    rise_time_end = ros::Time::now().toSec();
                    ROS_INFO("End recording Z axis rise time");

                }
                if (z - (initial_z + PID_TESTING_SETPOINT) > max_overshoot){
                    max_overshoot = z - (initial_z + PID_TESTING_SETPOINT);
                }
            }

            if (rise_time_end > 0){
                ROS_INFO_STREAM("Rise Time Z: " << (rise_time_end - rise_time_start));
                ROS_INFO_STREAM("Max Overshoot Z:  " << 100 * max_overshoot / PID_TESTING_SETPOINT  << "%");
            }
        }
        
        if (pid_tuning_xy && height_reached){
            if (x > initial_x + (0.1 *  PID_TESTING_SETPOINT) ){
                if (rise_time_start == -1){
                    rise_time_start = ros::Time::now().toSec();
                }
            }
            if (x > initial_x + (0.9 * PID_TESTING_SETPOINT)){
                if (rise_time_end == -1){
                    rise_time_end = ros::Time::now().toSec();
                }
                if (x - (initial_x + PID_TESTING_SETPOINT) > max_overshoot){
                    max_overshoot = x - (initial_x + PID_TESTING_SETPOINT);
                }
            }

            if (rise_time_end > 0){
                ROS_INFO_STREAM("Rise Time X:" << (rise_time_end - rise_time_start));
                ROS_INFO_STREAM("Max Overshoot X:  " << 100 * max_overshoot / PID_TESTING_SETPOINT << "%");
            }
        }
        //// IMPLEMENT /////
        
        // verbose
        if (verbose)
        {
            ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) FV(%6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", 
                     target_x, target_y, target_z, 
                     cmd_lin_vel_a, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
        }
        // wait for rate
        rate.sleep();
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}