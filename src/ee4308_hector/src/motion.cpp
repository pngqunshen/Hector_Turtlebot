#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // publish to pose topic
#include <geometry_msgs/Vector3Stamped.h>            // subscribe to magnetic topic
#include <sensor_msgs/Imu.h>                         // subscribe to imu topic
#include <sensor_msgs/NavSatFix.h>                   // subscribe to GPS
#include <hector_uav_msgs/Altimeter.h>               // subscribe to barometer
#include <sensor_msgs/Range.h>                       // subscribe to sonar
#include <nav_msgs/Odometry.h>                       // subscribe to ground truth topic
#include <std_srvs/Empty.h>                          // Service to calrbrate motors
#include <opencv2/core/core.hpp>
#include "common.hpp"

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, enable_baro, enable_magnet, enable_sonar, enable_gps;

// others
bool ready = false; // signal to topics to begin

// --------- PREDICTION WITH IMU ----------
const double G = 9.8;
double prev_imu_t = 0;
cv::Matx21d X = {0, 0}, Y = {0, 0}; // see intellisense. This is equivalent to cv::Matx<double, 2, 1>
cv::Matx21d A = {0, 0};
cv::Matx21d Z = {0, 0};
cv::Matx22d P_x = cv::Matx22d::ones(), P_y = cv::Matx22d::zeros();
cv::Matx22d P_a = cv::Matx22d::ones();
cv::Matx22d P_z = cv::Matx22d::ones();
double ua = NaN, ux = NaN, uy = NaN, uz = NaN;
double qa, qx, qy, qz;
// see https://docs.opencv.org/3.4/de/de1/classcv_1_1Matx.html
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!ready)
    {
        prev_imu_t = msg->header.stamp.toSec();
        return;
    }

    // calculate time
    double imu_t = msg->header.stamp.toSec();
    double imu_dt = imu_t - prev_imu_t;
    prev_imu_t = imu_t;

    // read inputs
    ua = msg->angular_velocity.z;
    ux = msg->linear_acceleration.x;
    uy = msg->linear_acceleration.y;
    uz = msg->linear_acceleration.z - G;
    
    //// IMPLEMENT IMU ////
    cv::Matx21d imu_acc = {ux, uy};
    // X
    cv::Matx22d F_xk = {
        1, imu_dt,
        0, 1
    };
    cv::Matx22d W_xk = {
        -0.5*pow(imu_dt, 2)*cos(A(0)), 0.5*pow(imu_dt, 2)*sin(A(0)),
        -imu_dt*cos(A(0))            , imu_dt*sin(A(0)) 
    };
    cv::Matx22d Qx = {
        qx, 0 ,
        0 , qy
    };
    X = F_xk*X + W_xk*imu_acc;
    P_x = F_xk*P_x*F_xk.t() + W_xk*Qx*W_xk.t();

    // Y
    cv::Matx22d F_yk = {
        1, imu_dt,
        0, 1
    };
    cv::Matx22d W_yk = {
        -0.5*pow(imu_dt, 2)*sin(A(0)),-0.5*pow(imu_dt, 2)*cos(A(0)),
        -imu_dt*sin(A(0))            ,-imu_dt*cos(A(0)) 
    };
    cv::Matx22d Qy = {
        qx, 0 ,
        0 , qy
    };
    Y = F_yk*Y + W_yk*imu_acc;
    P_y = F_yk*P_y*F_yk.t() + W_yk*Qy*W_yk.t();

    // Z
    cv::Matx22d F_zk = {
        1, imu_dt,
        0, 1
    };
    cv::Matx21d W_zk = {0.5*pow(imu_dt, 2), imu_dt};
    double Qz = qz;
    Z = F_zk*Z + W_zk*uz;
    P_z = F_zk*P_z*F_zk.t() + W_zk*Qz*W_zk.t();

    // A
    cv::Matx22d F_ak = {
        1, 0,
        0, 0
    };
    cv::Matx21d W_ak = {imu_dt, 1};
    double Qa = qa;
    A = F_ak*A + W_ak*ua;
    P_a = F_ak*P_a*F_ak.t() + W_ak*Qa*W_ak.t();
}

// --------- EKF CORRECTION ----------
void ekfCorrectionX(double Y_k, double h, cv::Matx12d H_k, double V_k, double R_k) {
    cv::Matx21d K_k = P_x*H_k.t() * 
        (1 / ((H_k*P_x*H_k.t())(0) + V_k*R_k*V_k));
    X = X + K_k*(Y_k - h);
    P_x = P_x - K_k*H_k*P_x;
}

void ekfCorrectionY(double Y_k, double h, cv::Matx12d H_k, double V_k, double R_k) {
    cv::Matx21d K_k = P_y*H_k.t() * 
        (1 / ((H_k*P_y*H_k.t())(0) + V_k*R_k*V_k));
    Y = Y + K_k*(Y_k - h);
    P_y = P_y - K_k*H_k*P_y;
}

void ekfCorrectionZ(double Y_k, double h, cv::Matx12d H_k, double V_k, double R_k) {
    cv::Matx21d K_k = P_z*H_k.t() * 
        (1 / ((H_k*P_z*H_k.t())(0) + V_k*R_k*V_k));
    Z = Z + K_k*(Y_k - h);
    P_z = P_z - K_k*H_k*P_z;
}

void ekfCorrectionYaw(double Y_k, double h, cv::Matx12d H_k, double V_k, double R_k) {
    cv::Matx21d K_k = P_a*H_k.t() * 
        (1 / ((H_k*P_a*H_k.t())(0) + V_k*R_k*V_k));
    A = A + K_k*(Y_k - h);
    P_a = P_z - K_k*H_k*P_a;
}

// --------- GPS ----------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
const double DEG2RAD = M_PI / 180;
const double RAD_POLAR = 6356752.3;
const double RAD_EQUATOR = 6378137;
double r_gps_x, r_gps_y, r_gps_z;

cv::Matx31d initial_ECEF = {NaN, NaN, NaN};
cv::Matx33d rot_m_n = {
    1,  0,  0,
    0, -1,  0,
    0,  0, -1
};
// EKF Correction Stuff for GPS
cv::Matx12d H_gps = {1, 0};
double V_gps = 1;

void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready)
        return;

    
    
    //// IMPLEMENT GPS /////
    double lat = msg->latitude; //IN DEGREES
    double lon = msg->longitude; //IN DEGREES
    double alt = msg->altitude;

    double lat_rad = DEG2RAD * lat;
    double lon_rad = DEG2RAD * lon;

    double eccentricity_sq = 1 - (pow(RAD_POLAR, 2) / pow(RAD_EQUATOR, 2));
    double n_phi = RAD_EQUATOR / sqrt(1 - (eccentricity_sq * pow(sin(lat_rad), 2)));
    
    cv::Matx31d ECEF = {
        (n_phi + alt) * cos(lat_rad) * cos(lon_rad),
        (n_phi + alt) * cos(lat_rad) * sin(lon_rad),
        ((pow(RAD_POLAR,2) / pow(RAD_EQUATOR, 2)) * n_phi + alt) * sin(lat_rad)
    };

    // for initial message -- you may need this:
    if (std::isnan(initial_ECEF(0)))
    {   // calculates initial ECEF and returns
        initial_ECEF = ECEF;
        return;
    }
    
    cv::Matx33d rot_e_n = {
        -sin(lat_rad) * cos(lon_rad), -sin(lon_rad), -cos(lat_rad) * cos(lon_rad),
        -sin(lat_rad) * sin(lon_rad), cos(lon_rad) , -cos(lat_rad) * sin(lon_rad),
         cos(lat_rad)               , 0            , -sin(lat_rad)
    };

    cv::Matx31d local_NED = rot_e_n.t() * (ECEF - initial_ECEF);

    GPS = rot_m_n * local_NED + initial_pos;

    double Y_gps_x = GPS(0,0);
    double Y_gps_y = GPS(1,0);
    double Y_gps_z = GPS(2,0);

    double h_gps_x = X(0,0);
    double h_gps_y = Y(0,0);
    double h_gps_z = Z(0,0);

    ekfCorrectionX(Y_gps_x, h_gps_x, H_gps, V_gps, r_gps_x);
    ekfCorrectionY(Y_gps_y, h_gps_y, H_gps, V_gps, r_gps_y);
    ekfCorrectionZ(Y_gps_z, h_gps_z, H_gps, V_gps, r_gps_z);
}

// --------- Magnetic ----------
double a_mgn = NaN;
double r_mgn_a;
cv::Matx12d H_mgn = {1, 0};
double V_mgn = 1;
void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;
   
    //// IMPLEMENT GPS ////
    double mx = msg->vector.x;
    double my = msg->vector.y;
    a_mgn = atan2(my, mx);
    double h_mgn_a = A(0,0);
    ekfCorrectionYaw(a_mgn, h_mgn_a, H_mgn, V_mgn, r_mgn_a);
}

// --------- Baro ----------
double z_bar = NaN;
double r_bar_z;
double V_bar = 1;
cv::Matx12d H_bar = {1, 0};
void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready)
        return;
    //// IMPLEMENT BARO ////
     z_bar = msg->altitude;
     double h_bar_z = Z(0,0);
     ekfCorrectionZ(z_bar, h_bar_z, H_bar, V_bar, r_bar_z);
}

// --------- Sonar ----------
double z_snr = NaN;
double r_snr_z;
// EKF Correction Stuff for GPS
cv::Matx12d H_snr = {1, 0};
double V_snr = 1;
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT SONAR ////
    z_snr = msg->range;
    
    double h_snr = Z(0,0);
    ekfCorrectionZ(z_snr, h_snr, H_snr, V_snr, r_snr_z);  
}

// --------- GROUND TRUTH ----------
nav_msgs::Odometry msg_true;
void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

// --------- MEASUREMENT UPDATE WITH GROUND TRUTH ----------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_motion");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    double motion_iter_rate;
    if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        ROS_WARN("HMOTION: Param motion_iter_rate not found, set to 50.0");
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN("HMOTION: Param verbose_motion not found, set to false");
    if (!nh.param("initial_x", X(0), 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", Y(0), 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", Z(0), 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    initial_pos = {X(0), Y(0), Z(0)};
    if (!nh.param("use_ground_truth", use_ground_truth, true))
        ROS_WARN("HMOTION: Param use_ground_truth not found, set use_ground_truth to true");
    if (!nh.param("r_gps_x", r_gps_x, 1.0))
        ROS_WARN("HMOTION: Param r_gps_x not found, set to 1.0");
    if (!nh.param("r_gps_y", r_gps_y, 1.0))
        ROS_WARN("HMOTION: Param r_gps_y not found, set to 1.0");
    if (!nh.param("r_gps_z", r_gps_z, 1.0))
        ROS_WARN("HMOTION: Param r_gps_z not found, set to 1.0");
    if (!nh.param("r_mgn_a", r_mgn_a, 1.0))
        ROS_WARN("HMOTION: Param r_mgn_a not found, set to 1.0");
    if (!nh.param("r_bar_z", r_bar_z, 1.0))
        ROS_WARN("HMOTION: Param r_bar_z not found, set to 1.0");
    if (!nh.param("r_snr_z", r_snr_z, 1.0))
        ROS_WARN("HMOTION: Param r_snr_z not found, set to 1.0");
    if (!nh.param("qa", qa, 1.0))
        ROS_WARN("HMOTION: Param qa not found, set to 1.0");
    if (!nh.param("qx", qx, 1.0))
        ROS_WARN("HMOTION: Param qx not found, set to 1.0");
    if (!nh.param("qy", qy, 1.0))
        ROS_WARN("HMOTION: Param qy not found, set to 1.0");
    if (!nh.param("qz", qz, 1.0))
        ROS_WARN("HMOTION: Param qz not found, set to 1.0");
    if (!nh.param("enable_baro", enable_baro, true))
        ROS_WARN("HMOTION: Param enable_baro not found, set to true");
    if (!nh.param("enable_magnet", enable_magnet, true))
        ROS_WARN("HMOTION: Param enable_magnet not found, set to true");
    if (!nh.param("enable_sonar", enable_sonar, true))
        ROS_WARN("HMOTION: Param enable_sonar not found, set to true");
    if (!nh.param("enable_gps", enable_gps, true))
        ROS_WARN("HMOTION: Param enable_gps not found, set to true");

    // --------- Subscribers ----------
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw_imu", 1, &cbImu);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("fix", 1, &cbGps);
    if (!enable_gps)
        sub_gps.shutdown();
    ros::Subscriber sub_magnet = nh.subscribe<geometry_msgs::Vector3Stamped>("magnetic", 1, &cbMagnet);
    if (!enable_magnet)
        sub_magnet.shutdown();
    ros::Subscriber sub_baro = nh.subscribe<hector_uav_msgs::Altimeter>("altimeter", 1, &cbBaro);
    if (!enable_baro)
        sub_baro.shutdown();
    ros::Subscriber sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &cbSonar);
    if (!enable_sonar)
        sub_sonar.shutdown();

    // --------- Publishers ----------
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";   // for rviz
    msg_pose.pose.pose.orientation.x = 0; // no roll
    msg_pose.pose.pose.orientation.y = 0; // no pitch
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocity", 1, true); // publish velocity
    geometry_msgs::Twist msg_vel;

    // --------- Wait for Topics ----------
    ROS_INFO("HMOTION: Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ((std::isnan(ux) && msg_true.header.seq == 0))) // wait for imu and truth only
        ros::spinOnce(); // update subscribers

    if (!ros::ok())
    { // ROS shutdown
        ROS_INFO("HMOTION: ===== END =====");
        return 0;
    }

    // --------- Calibrate Gyro service ----------
    ROS_INFO("HMOTION: Calibrating Gyro...");
    ros::ServiceClient calibrate_gyro = nh.serviceClient<std_srvs::Empty>("raw_imu/calibrate");
    std_srvs::Empty calibrate_gyro_srv;
    if (calibrate_gyro.call(calibrate_gyro_srv))
        ROS_INFO("HMOTION: Calibrated Gyro");
    else
        ROS_WARN("HMOTION: Gyro cannot be calibrated!");

    // --------- Main loop ----------

    ros::Rate rate(motion_iter_rate);
    ROS_INFO("HMOTION: ===== BEGIN =====");
    ready = true;
    while (ros::ok() && nh.param("run", true))
    {
        ros::spinOnce(); // update topics

        // Verbose
        if (verbose)
        {
            auto & tp = msg_true.pose.pose.position;
            auto &q = msg_true.pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]  TRUE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", tp.x, tp.y, tp.z, atan2(siny_cosp, cosy_cosp));
            ROS_INFO("[HM] STATE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", X(0), Y(0), Z(0), A(0));
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", GPS(0), GPS(1), GPS(2));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", a_mgn);
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", z_bar);
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z(3));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", z_snr);
        }

        //  Publish pose and vel
        if (use_ground_truth)
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position = msg_true.pose.pose.position;
            msg_pose.pose.pose.orientation = msg_true.pose.pose.orientation;
            msg_vel = msg_true.twist.twist;
        }
        else
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position.x = X(0);
            msg_pose.pose.pose.position.y = Y(0);
            msg_pose.pose.pose.position.z = Z(0);
            msg_pose.pose.covariance[0] = P_x(0, 0);  // x cov
            msg_pose.pose.covariance[7] = P_y(0, 0);  // y cov
            msg_pose.pose.covariance[14] = P_z(0, 0); // z cov
            msg_pose.pose.covariance[35] = P_a(0, 0); // a cov
            msg_pose.pose.pose.orientation.w = cos(A(0) / 2);
            msg_pose.pose.pose.orientation.z = sin(A(0) / 2);
            msg_vel.linear.x = X(1);
            msg_vel.linear.y = Y(1);
            msg_vel.linear.z = Z(1);
            msg_vel.angular.z = A(1);
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        rate.sleep();
    }

    ROS_INFO("HMOTION: ===== END =====");
    return 0;
}

