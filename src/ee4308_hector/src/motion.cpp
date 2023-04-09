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
#include <nav_msgs/OccupancyGrid.h>                  // subscribe to turtle inflation
#include <opencv2/core/core.hpp>
#include "common.hpp"

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, debug, enable_baro, enable_magnet, enable_sonar, enable_gps;

// others
bool ready = false; // signal to topics to begin

// --------- INFLATION FROM TURTLE ----------
nav_msgs::OccupancyGrid turtle_inflation;
void cbTurtleInf(const nav_msgs::OccupancyGrid &msg)
{
    turtle_inflation = msg;
}

// --------- PREDICTION WITH IMU ----------
const double G = 9.8;
double prev_imu_t = 0; // track previous IMU time so dt can be calculated
cv::Matx21d X = {0, 0}, Y = {0, 0}; // see intellisense. This is equivalent to cv::Matx<double, 2, 1>
cv::Matx21d A = {0, 0};
cv::Matx31d Z = {0, 0, 0}; // changed to 3x1 vector to account for baro bias
cv::Matx22d P_x = cv::Matx22d::ones(), P_y = cv::Matx22d::ones();
cv::Matx22d P_a = cv::Matx22d::ones();
cv::Matx33d P_z = cv::Matx33d::ones(); // changed to 3x3 matrix to account for baro bias
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
    uz = msg->linear_acceleration.z - G; // subtract gravitatational acceleration
    
    //// IMPLEMENT IMU ////
    cv::Matx21d imu_acc = {ux, uy}; // 2x1 vextor of x and y IMU acceleration reading
    /////////////////////////////////////
    // X
    cv::Matx22d F_xk = { // jacobian matrix with respect to previous x state
        1, imu_dt,
        0, 1
    };
    cv::Matx22d W_xk = { // jacobian matrix with respect to previous IMU reading in x
        -0.5*pow(imu_dt, 2)*cos(A(0)), 0.5*pow(imu_dt, 2)*sin(A(0)),
        -imu_dt*cos(A(0))            , imu_dt*sin(A(0)) 
    };
    cv::Matx22d Qx = { // diagonal covariance matrix of the IMU noise in x and y
        qx, 0 ,
        0 , qy
    };
    X = F_xk*X + W_xk*imu_acc; // EKF prediction of X from IMU
    P_x = F_xk*P_x*F_xk.t() + W_xk*Qx*W_xk.t(); // filtered covariance matrix of X from IMU
    /////////////////////////////////////

    /////////////////////////////////////
    // Y
    cv::Matx22d F_yk = { // jacobian matrix with respect to previous IMU y state
        1, imu_dt,
        0, 1
    };
    cv::Matx22d W_yk = { // jacobian matrix with respect to previous IMU reading in y
        -0.5*pow(imu_dt, 2)*sin(A(0)),-0.5*pow(imu_dt, 2)*cos(A(0)),
        -imu_dt*sin(A(0))            ,-imu_dt*cos(A(0)) 
    };
    cv::Matx22d Qy = { // diagonal covariance matrix of the IMU noise in x and y
        qx, 0 ,
        0 , qy
    };
    Y = F_yk*Y + W_yk*imu_acc; // EKF prediction of Y from IMU
    P_y = F_yk*P_y*F_yk.t() + W_yk*Qy*W_yk.t(); // filtered covariance matrix of Y from IMU
    /////////////////////////////////////

    /////////////////////////////////////
    // Z
    cv::Matx33d F_zk = { // jacobian matrix with respect to previous IMU z state
        1, imu_dt, 0,
        0, 1     , 0,
        0, 0     , 1    
    };
    // jacobian matrix with respect to previous IMU reading in z
    cv::Matx31d W_zk = {0.5*pow(imu_dt, 2), imu_dt, 0};
    double Qz = qz; // diagonal covariance matrix of the IMU noise in z
    Z = F_zk * Z + W_zk * uz; // EKF prediction of Z from IMU
    P_z = F_zk*P_z*F_zk.t() + W_zk*Qz*W_zk.t(); // filtered covariance matrix of Z from IMU
    /////////////////////////////////////

    /////////////////////////////////////
    // A
    cv::Matx22d F_ak = { // jacobian matrix with respect to previous IMU angular state
        1, 0,
        0, 0
    };
    // jacobian matrix with respect to previous IMU reading in angular
    cv::Matx21d W_ak = {imu_dt, 1};
    double Qa = qa; // diagonal covariance matrix of the IMU noise in angular
    A = F_ak*A + W_ak*ua; // EKF prediction of angular from IMU
    P_a = F_ak*P_a*F_ak.t() + W_ak*Qa*W_ak.t(); // filtered covariance matrix of angular from IMU
    /////////////////////////////////////
}

// --------- EKF CORRECTION ----------
// EKF correction in X. 1x1 matrices are taken in double to simplify calculation
void ekfCorrectionX(double Y_k, double h, cv::Matx12d H_k, double V_k, 
                    double R_k, double bias) {
    cv::Matx21d K_k = P_x*H_k.t() * 
        (1 / ((H_k*P_x*H_k.t())(0) + V_k*R_k*V_k));
    X = X + K_k*(Y_k - h - bias);
    P_x = P_x - K_k*H_k*P_x;
}

// EKF correction in Y. 1x1 matrices are taken in double to simplify calculation
void ekfCorrectionY(double Y_k, double h, cv::Matx12d H_k, double V_k, 
                    double R_k, double bias) {
    cv::Matx21d K_k = P_y*H_k.t() * 
        (1 / ((H_k*P_y*H_k.t())(0) + V_k*R_k*V_k));
    Y = Y + K_k*(Y_k - h - bias);
    P_y = P_y - K_k*H_k*P_y;
}

// EKF correction in Z. 1x1 matrices are taken in double to simplify calculation
void ekfCorrectionZ(double Y_k, double h, cv::Matx13d H_k, double V_k, 
                    double R_k, double bias) {
    cv::Matx31d K_k = P_z*H_k.t() * 
        (1 / ((H_k*P_z*H_k.t())(0) + V_k*R_k*V_k));
    Z = Z + K_k*(Y_k - h - bias);
    P_z = P_z - K_k*H_k*P_z;
}

// EKF correction in yaw. 1x1 matrices are taken in double to simplify calculation
void ekfCorrectionYaw(double Y_k, double h, cv::Matx12d H_k, double V_k, 
                      double R_k, double bias) {
    cv::Matx21d K_k = P_a*H_k.t() * 
        (1 / ((H_k*P_a*H_k.t())(0) + V_k*R_k*V_k));
    A = A + K_k*(Y_k - h - bias);
    P_a = P_a - K_k*H_k*P_a;
}

// for variance calculation
std::vector<float> sonar_data; // variance date for sonar
std::vector<float> baro_data; // variance date for baro
std::vector<float> mag_data; // variance date for magnetometer

// --------- GPS ----------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
const double DEG2RAD = M_PI / 180; // to convert degree to radian
const double RAD_POLAR = 6356752.3; // polar radius of Earth
const double RAD_EQUATOR = 6378137; // equatorial radius of Earth
double r_gps_x, r_gps_y, r_gps_z;

cv::Matx31d initial_ECEF = {NaN, NaN, NaN};
// rotation matrix from local NED to Gazebo world frame
cv::Matx33d rot_m_n = {
    1,  0,  0,
    0, -1,  0,
    0,  0, -1
};
// EKF Correction Stuff for GPS
cv::Matx12d H_gps = {1, 0}; // H matrix for GPS for x and y
cv::Matx13d H_gps_z = {1, 0, 0}; // H matrix for GPS for z as baro bias is included
double V_gps = 1; // V matrix for gps, 1x1 matrix so double is used

void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready)
        return;

    // //// IMPLEMENT GPS /////
    double lat = msg->latitude; //IN DEGREES
    double lon = msg->longitude; //IN DEGREES
    double alt = msg->altitude;

    double lat_rad = DEG2RAD * lat; // convert lat measurements to radians
    double lon_rad = DEG2RAD * lon; // convert lon measurements to radians

    double eccentricity_sq = 1 - (pow(RAD_POLAR, 2) / pow(RAD_EQUATOR, 2));
    double n_phi = RAD_EQUATOR / sqrt(1 - (eccentricity_sq * pow(sin(lat_rad), 2)));
    
    // GPS readings in ECEF coordinates
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
    
    // rotation matrix from ECEF to local NED frame
    cv::Matx33d rot_e_n = {
        -sin(lat_rad) * cos(lon_rad), -sin(lon_rad), -cos(lat_rad) * cos(lon_rad),
        -sin(lat_rad) * sin(lon_rad),  cos(lon_rad), -cos(lat_rad) * sin(lon_rad),
         cos(lat_rad)               ,  0           , -sin(lat_rad)
    };

    // GPS readings in local NED frame
    cv::Matx31d local_NED = rot_e_n.t() * (ECEF - initial_ECEF);

    // GPS readings in Gazebo world frame
    GPS = rot_m_n * local_NED + initial_pos;

    double Y_gps_x = GPS(0,0); // final reading x
    double Y_gps_y = GPS(1,0); // final reading y
    double Y_gps_z = GPS(2,0); // final reading z

    double h_gps_x = X(0,0);
    double h_gps_y = Y(0,0);
    double h_gps_z = Z(0,0);

    // EKF correction
    ekfCorrectionX(Y_gps_x, h_gps_x, H_gps, V_gps, r_gps_x, 0);
    ekfCorrectionY(Y_gps_y, h_gps_y, H_gps, V_gps, r_gps_y, 0);
    ekfCorrectionZ(Y_gps_z, h_gps_z, H_gps_z, V_gps, r_gps_z, 0);
}

// --------- Magnetic ----------
double a_mgn = NaN;
double r_mgn_a;
cv::Matx12d H_mgn = {1, 0}; // H matrix for magnetometer for yaw
double V_mgn = 1; // V matrix for magnetometer, 1x1 matrix so double is used
void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;
   
    //// IMPLEMENT MAG ////
    double mx = msg->vector.x;
    double my = msg->vector.y;
    a_mgn = -atan2(my, mx); // negative atan2 to convert reading into heading
    double h_mgn_a = A(0,0);

    // EKF correction
    ekfCorrectionYaw(a_mgn, h_mgn_a, H_mgn, V_mgn, r_mgn_a, 0);

    // for variance calculation
    if (debug) {
        mag_data.push_back(a_mgn);
        ROS_INFO_STREAM("number of mag data: " << mag_data.size());
        if (mag_data.size() == 500)
        {
            double sum = 0;
            for (int i = 0; i < mag_data.size(); i++)
            {
                sum += mag_data[i];
            }
            double mean = sum / mag_data.size();
            double sum_sq = 0;
            for (int i = 0; i < mag_data.size(); i++)
            {
                sum_sq += pow(mag_data[i] - mean, 2);
            }
            double variance = sum_sq / (mag_data.size() - 1);
            for (int i = 0; i < mag_data.size(); i++)
            {
                std::cout << mag_data[i] << ", ";     
            }
            ROS_INFO_STREAM("mag variance: " << variance);
        }
    }
}

// --------- Baro ----------
double z_bar = NaN;
// EKF Correction Stuff for Baro
double r_bar_z;
cv::Matx13d H_bar = {1, 0, 1}; // H matrix for barometer for z, including bias
double V_bar = 1; // V matrix for barometer, 1x1 matrix so double is used
void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready) {
        return;
    }
    //// IMPLEMENT BARO ////
     z_bar = msg->altitude;
     double h_bar_z = Z(0,0);

     // EKF correction
     // bias is 3rd term in the Z state
     ekfCorrectionZ(z_bar, h_bar_z, H_bar, V_bar, r_bar_z, Z(2));
    
    // for variance calculation
    if (debug) {
        baro_data.push_back(z_bar);
        ROS_INFO_STREAM("number of baro data: " << baro_data.size());
        if (baro_data.size() == 500)
        {
            double sum = 0;
            for (int i = 0; i < baro_data.size(); i++)
            {
                sum += baro_data[i];
            }
            double mean = sum / baro_data.size();
            double sum_sq = 0;
            for (int i = 0; i < baro_data.size(); i++)
            {
                sum_sq += pow(baro_data[i] - mean, 2);
            }
            double variance = sum_sq / (baro_data.size() - 1);
            for (int i = 0; i < baro_data.size(); i++)
            {
                std::cout << baro_data[i] << ", ";
            }
            ROS_INFO_STREAM("baro variance: " << variance);
        }
    }
}

// --------- Sonar ----------
// function that checks whether hector is flying over obstacles from turtle occupancy grid
// detects obstacle but sonar readings still a bit off nearer to walls
// occasionally crashes motion node, to investigate why
bool notOverObstacle(void)
{
    // convert current hector position to index for occupancy grid
    int ind_x = round((X(0) - turtle_inflation.info.origin.position.x)/turtle_inflation.info.resolution);
    int ind_y = round((Y(0) - turtle_inflation.info.origin.position.y)/turtle_inflation.info.resolution);
    // check if hector in turtle's inflation zone
    if (turtle_inflation.data[turtle_inflation.info.width*ind_x+ind_y] == 1)
    {
        ROS_INFO("Flying over obstacle!!");
        return false;
    }
    return true;
}

double z_snr = NaN;
double r_snr_z;
// EKF Correction Stuff for GPS
cv::Matx13d H_snr = {1, 0, 0}; // H matrix for sonar for z, including baro bias
double V_snr = 1; // V matrix for barometer, 1x1 matrix so double is used
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT SONAR ////
    z_snr = msg->range;
    
    double h_snr = Z(0,0);

    // EKF correction
    if (notOverObstacle())
    {
        ekfCorrectionZ(z_snr, h_snr, H_snr, V_snr, r_snr_z, 0);  
    }

    // for variance calculation
    if (debug) {
        sonar_data.push_back(z_snr);
        ROS_INFO_STREAM("number of sonar data: " << sonar_data.size());
        if (sonar_data.size() == 500)
        {
            double sum = 0;
            for (int i = 0; i < sonar_data.size(); i++)
            {
                sum += sonar_data[i];
            }
            double mean = sum / sonar_data.size();
            double sum_sq = 0;
            for (int i = 0; i < sonar_data.size(); i++)
            {
                sum_sq += pow(sonar_data[i] - mean, 2);
            }
            double variance = sum_sq / (sonar_data.size() - 1);
            for (int i = 0; i < sonar_data.size(); i++)
            {
                std::cout << sonar_data[i] << ", ";
            }
            ROS_INFO_STREAM("sonar variance: " << variance);
        }
    }
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
    if (!nh.param("debug_motion", debug, false))
        ROS_WARN("HMOTION: Param debug_motion not found, set to false");
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
    ros::Subscriber sub_t_inf = nh.subscribe("/turtle/grid/inflation", 1, &cbTurtleInf);

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

        // for comparing with ground truth
        double rmseX = 0;
        double rmseY = 0;
        double rmseZ = 0;
        double rmseA = 0;
        double rmseLin = 0;
        int counter = 0;

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
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z(2));
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

            if (debug && !use_ground_truth)
            {
                double siny_cosp_gt = 2 * (msg_true.pose.pose.orientation.w * msg_true.pose.pose.orientation.z + msg_true.pose.pose.orientation.x * msg_true.pose.pose.orientation.y);
                double cosy_cosp_gt = 1 - 2 * (msg_true.pose.pose.orientation.y * msg_true.pose.pose.orientation.y + msg_true.pose.pose.orientation.z * msg_true.pose.pose.orientation.z);

                counter++;
                rmseX += pow(msg_true.pose.pose.position.x - X(0), 2);
                rmseY += pow(msg_true.pose.pose.position.y - Y(0), 2);
                rmseLin += pow(msg_true.pose.pose.position.x - X(0), 2) + pow(msg_true.pose.pose.position.y - Y(0), 2);
                rmseZ += pow(msg_true.pose.pose.position.z - Z(0), 2);
                rmseA += pow(atan2(siny_cosp_gt, cosy_cosp_gt) - A(0), 2);

                ROS_INFO_STREAM("RMSE X: " << sqrt(rmseX / counter));
                ROS_INFO_STREAM("RMSE Y: " << sqrt(rmseY / counter));
                ROS_INFO_STREAM("RMSE LINEAR: " << sqrt(rmseLin / counter));
                ROS_INFO_STREAM("RMSE Z: " << sqrt(rmseZ / counter));
                ROS_INFO_STREAM("RMSE A: " << sqrt(rmseA / counter));
            }
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        rate.sleep();
    }

    ROS_INFO("HMOTION: ===== END =====");
    return 0;
}