#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

bool verbose;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;
void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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
double turtle_x = NaN, turtle_y = NaN;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}
nav_msgs::Path turtle_traj;
bool replan = true;
void cbTTraj(const nav_msgs::Path &msg){
    turtle_traj = msg;
    replan = true;
}

class Trajectory{
    public:
    Trajectory(){};
    
    void init_traj(Position3d pos_begin, Position3d pos_end, Position3d curr_speed, double average_speed, double desired_dt){
        double Dx = pos_end.x - pos_begin.x;
        double Dy = pos_end.y - pos_begin.y;
        double Dz = pos_end.z - pos_begin.z;
        duration = sqrt(Dx*Dx + Dy*Dy + Dz*Dz) / average_speed;

        xCoeff = {0,0,0,0,0,0};
        yCoeff = {0,0,0,0,0,0};
        zCoeff = {0,0,0,0,0,0};
        
        cv::Matx66d Minv = {1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 0.5, 0, 0, 0,
        -10/pow(duration, 3), -6/pow(duration,2), -3/(2*duration), 10/pow(duration,3), -4/pow(duration,2), 1/(2*duration),
        15/pow(duration,4), 8/pow(duration,3), 3/(2*pow(duration,2)), -15/pow(duration,4), 7/pow(duration,3), -1/pow(duration,2),
        -6/pow(duration,5), -3/pow(duration,4), -1/(2*pow(duration,3)), 6/pow(duration,5), -3/pow(duration,4), 1/(2*pow(duration,3))
        };

        cv::Vec6d x = {pos_begin.x, curr_speed.x, 0, pos_end.x, 0, 0};
        cv::Vec6d y = {pos_begin.y, curr_speed.y, 0, pos_end.y, 0, 0};
        cv::Vec6d z = {pos_begin.z, curr_speed.z, 0, pos_end.z, 0, 0};

        xCoeff = Minv * x;
        yCoeff = Minv * y;
        zCoeff = Minv * z;

        path.poses.clear();
        path.header.frame_id = "world"; 
        
        for (int i = 0; i < duration / desired_dt; i++){

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            for (int j = 0; j < 6; j++){
                pose.pose.position.x += xCoeff[j] * pow(desired_dt * i,j);
                pose.pose.position.y += yCoeff[j] * pow(desired_dt * i,j);
                pose.pose.position.z += zCoeff[j] * pow(desired_dt * i,j);

            }
            path.poses.push_back(pose);
        }
    };

    Position3d get_next_goal(double idx){
        
        Position3d target;
        if (idx > path.poses.size() - 1){
            idx = path.poses.size() - 1;
        }
        target.x = path.poses[idx].pose.position.x;
        target.y = path.poses[idx].pose.position.y;
        target.z = path.poses[idx].pose.position.z;

        return target;
    };

    cv::Vec6d xCoeff;
    cv::Vec6d yCoeff;
    cv::Vec6d zCoeff;
    double duration;
    nav_msgs::Path path;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    double average_speed_z;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("average_speed_z", average_speed_z, 0.5))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 0.5");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;
    if (nh.param("/turtle/goals", goal_str, std::to_string(initial_x) + "," + std::to_string(initial_y))) // set to initial hector positions
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goal_x = strtod(tok, nullptr);
                goal_y = strtod(strtok(nullptr, " ,"), nullptr);
                tok = strtok(nullptr, " ,");
            }
            ROS_INFO(" HMAIN : Last Turtle Goal is (%lf, %lf)", goal_x, goal_y);
        }
        catch (...)
        {
            ROS_ERROR(" HMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);
    ros::Subscriber sub_ttraj = nh.subscribe("/turtle/trajectory", 1, &cbTTraj);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    HectorState state = TAKEOFF;
    ros::Rate rate(main_iter_rate);
    Position3d target = {0,0,0};
    Trajectory traj;
    int look_ahead_time = (look_ahead / average_speed) * main_iter_rate;
    int look_ahead_time_z = (look_ahead / average_speed_z) * main_iter_rate;

    bool traj_init = false;
    double t;
    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();

        // remove this block comment (1/2)
        //// IMPLEMENT ////
        if (state == TAKEOFF)
        {   // Initial State
            // Disable Rotate
            msg_rotate.data = false;    
            pub_rotate.publish(msg_rotate);

            if (!traj_init){
                // this does not use hector state to generate traj, this is because the hector estimated state takes a few iterations to converge to a reasonable value. this may cause crazy trajs 
                traj.init_traj(Position3d(initial_x, initial_y, initial_z), Position3d(initial_x, initial_y, height), Position3d(0, 0, 0), average_speed_z, 1/main_iter_rate);
                pub_traj.publish(traj.path);
                
                if (traj.path.poses.size() > look_ahead_time_z - 1){
                    t = look_ahead_time_z;
                } else {
                    t = traj.path.poses.size() - 1;
                }
                traj_init = true;
            }

            if (dist_euc(Position3d(x,y,z), target) < look_ahead){
                if (t + 1 < traj.path.poses.size()){
                    t += 1;
                } else {
                    t = traj.path.poses.size() - 1;
                }
            }

            target = traj.get_next_goal(t);

            if (dist_euc(Position3d(x,y,z), Position3d(initial_x, initial_y, height)) < close_enough){
                state = TURTLE;
                traj_init = false;
            }

            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = target.z;
            pub_target.publish(msg_target);
            // Enable Rotate when the height is reached
        }
        else if (state == TURTLE)
        {   // flying to turtlebot
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);



            if (!traj_init){
                traj.init_traj(Position3d(x,y,z), Position3d(turtle_x, turtle_y, height), Position3d(vx, vy, 0), average_speed, 1/main_iter_rate);
                pub_traj.publish(traj.path);
                
                if (traj.path.poses.size() > look_ahead_time - 1){
                    t = look_ahead_time;
                } else {
                    t = traj.path.poses.size() - 1;
                }
                traj_init = true;
            }

            if (dist_euc(Position3d(x,y,z), target) < look_ahead){
                if (t + 1 < traj.path.poses.size()){
                    t += 1;
                } else {
                    t = traj.path.poses.size() - 1;
                }
            }

            target = traj.get_next_goal(t);
            if (dist_euc(Position3d(x,y,z), Position3d(turtle_x, turtle_y, height)) < close_enough){
                state = GOAL;
                traj_init = false;
            }

            if (dist_euc(traj.get_next_goal(traj.path.poses.size() - 1), Position3d(turtle_x, turtle_y, height)) > close_enough){
                traj_init = false;
            }

            if (dist_euc(Position3d(x,y,z), Position3d(turtle_x, turtle_y, height)) < look_ahead){
                target = Position3d(turtle_x, turtle_y, height);
                traj_init = false;
            }

            
            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = height;
            pub_target.publish(msg_target);
        }
        else if (state == START)
        {   // flying to hector's starting position
            
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);

            if (!traj_init){
                traj.init_traj(Position3d(x,y,z), Position3d(initial_x, initial_y, height), Position3d(vx, vy, 0), average_speed, 1/main_iter_rate);
                pub_traj.publish(traj.path);
                
                if (traj.path.poses.size() > look_ahead_time - 1){
                    t = look_ahead_time;
                } else {
                    t = traj.path.poses.size() - 1;
                }
                traj_init = true;
            }

            if (dist_euc(Position3d(x,y,z), target) < look_ahead){
                if (t + 1 < traj.path.poses.size()){
                    t += 1;
                } else {
                    t = traj.path.poses.size() - 1;
                }
            }
            target = traj.get_next_goal(t);

            if (dist_euc(Position3d(x,y,z), Position3d(initial_x, initial_y, height)) < close_enough){
                if (!nh.param("/turtle/run", false))
                { // use this if else case to track when the turtle reaches the final goal
                    state = LAND;
                }
                else
                {
                    state = TURTLE;
                }
                traj_init = false;
            }
            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = height;
            pub_target.publish(msg_target);
        }
        else if (state == GOAL)
        {   // flying to goal
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);

            if (!traj_init){
                traj.init_traj(Position3d(x,y,z), Position3d(goal_x, goal_y, height), Position3d(vx, vy, 0), average_speed, 1/main_iter_rate);
                pub_traj.publish(traj.path);
                
                if (traj.path.poses.size() > look_ahead_time - 1){
                    t = look_ahead_time;
                } else {
                    t = traj.path.poses.size() - 1;
                }
                traj_init = true;
            }

            if (dist_euc(Position3d(x,y,z), target) < look_ahead){
                if (t + 1 < traj.path.poses.size()){
                    t += 1;
                } else {
                    t = traj.path.poses.size() - 1;
                }
            }

            target = traj.get_next_goal(t);

            if (dist_euc(Position3d(x,y,z), Position3d(goal_x, goal_y, height)) < close_enough){
                state = START;
                traj_init = false;
            }

            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = height;
            pub_target.publish(msg_target);
        }
        else if (state == LAND)
        {   // reached hector's starting position, and trying to land. Can disable rotation.
            msg_rotate.data = false;
            pub_rotate.publish(msg_rotate);

            if (!traj_init){
                traj.init_traj(Position3d(x,y,z), Position3d(initial_x, initial_y, initial_z), Position3d(vx, vy, 0), average_speed_z, 1/main_iter_rate);
                pub_traj.publish(traj.path);
                
                if (traj.path.poses.size() > look_ahead_time_z - 1){
                    t = look_ahead_time_z;
                } else {
                    t = traj.path.poses.size() - 1;
                }
                traj_init = true;
            }

            target = traj.get_next_goal(t);

            if (dist_euc(Position3d(x,y,z), target) < look_ahead){
                if (t + 1 < traj.path.poses.size()){
                    t += 1;
                } else {
                    t = traj.path.poses.size() - 1;
                }
            }

            if (dist_euc(Position3d(x,y,z), Position3d(initial_x, initial_y, initial_z)) < close_enough){
                ROS_INFO(" Reached Last Target");
                break;
            }
            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = target.z;
            pub_target.publish(msg_target);
            
        }

        // remove this block comment (2/2)

        if (verbose)
            ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}
