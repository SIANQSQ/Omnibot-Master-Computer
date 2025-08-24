#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "robo_utils/ChassisMotionControl.h"

#include <Eigen/Dense>

#include <memory>
#include <cmath>

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#pragma pack(push, 1)
struct PosStruct {
    float x;
    float y;
    float qw;
    float qx;
    float qy;
    float qz;
};
#pragma pack(pop)




char ip[]="192.168.1.50";
std::shared_ptr<tf::TransformListener> listener_p;

// 初始化PID参数
double kp = 1.8, ki = 0.024;
// double kp = 0.5, ki = 0.005;
Eigen::Vector2d integral;

double kp_yaw = 1.4, ki_yaw = 0.0;
double integral_yaw;

// 位置 位置期望
Eigen::Vector2d position, goal_position;
Eigen::Quaterniond quaternion, goal_quaternion;

bool inited = false;

double getYawFromQuaternion(const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr msg_p) {
    position = Eigen::Vector2d(msg_p->pose.pose.position.x, msg_p->pose.pose.position.y);
    quaternion = Eigen::Quaterniond(msg_p->pose.pose.orientation.w, msg_p->pose.pose.orientation.x, msg_p->pose.pose.orientation.y, msg_p->pose.pose.orientation.z);
    if (!inited) {
        goal_position = position;
        goal_quaternion = quaternion;
        inited = true;
    }
}

void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr msg_p) {
    goal_position = Eigen::Vector2d(msg_p->pose.position.x, msg_p->pose.position.y);
    goal_quaternion = Eigen::Quaterniond(msg_p->pose.orientation.w, msg_p->pose.orientation.x, msg_p->pose.orientation.y, msg_p->pose.orientation.z);
    // ROS_INFO("Goal was set at position %f,%f", msg_p->pose.position.x, msg_p->pose.position.y); 
}


bool recdata()
{
    // 创建TCP套接字
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        perror("socket creation failed");
        return 1;
    }

    // 设置服务器地址
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) {
        perror("invalid address");
        close(sock);
        return 1;
    }

    // 连接服务器
        if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connection failed");
        close(sock);
        return 1;
        sleep(1000);
    }

    std::cout << "Connected to server at 192.168.1.50:" << 12345 << std::endl;

    while (true) {
        // 接收服务端请求
        PosStruct request;
        int bytes_received = recv(sock, &request, sizeof(PosStruct), 0);
        if (bytes_received <= 0) {
            if (bytes_received == 0) {
                std::cout << "Server disconnected" << std::endl;
            } else {
                perror("recv failed");
            }
            break;
        }

        // 显示服务端请求
        std::cout << "Received request from server: "
                  << "x=" << request.x << ", y=" << request.y << ", qw=" << request.qw << ", qx=" << request.qx << ", qy=" << request.qy << ", qz=" << request.qz <<std::endl;
/*

>>>>>接收到一次数据后，运行这里的代码<<<<<
*/  
    goal_position.x()=request.x;
    goal_position.y()=request.y;
    goal_quaternion.w()=request.qw;
    goal_quaternion.x()=request.qx;
    goal_quaternion.y()=request.qy;
    goal_quaternion.z()=request.qz;
    std::cout<<"ok"<<std::endl;
    }
    close(sock);
    return 0;
}

void control(ros::Publisher &pub) {
        //PID计算
        Eigen::Vector2d error, out;
        double error_yaw = getYawFromQuaternion(goal_quaternion) - getYawFromQuaternion(quaternion);
        double out_yaw;

        while (error_yaw > M_PI) {
            error_yaw -= 2 * M_PI;
        }

        while (error_yaw < -M_PI) {
            error_yaw += 2 * M_PI;
        }

        error = (quaternion.conjugate().toRotationMatrix() * (
                Eigen::Vector3d(goal_position.x(), goal_position.y(), 0) - Eigen::Vector3d(position.x(), position.y(), 0)
            )).head<2>();

        if (error.norm() < 0.15) {
            out = Eigen::Vector2d(0, 0);
        } else {
            integral += error;

            if (integral.norm() > 8.0) {
                integral = integral.normalized() * 8.0;
            }

            out = kp * error + ki * integral;

            if (out.norm() > 1.8) {
                out = out.normalized() * 1.8;
            }
        }

        if (fabs(error_yaw) < 0.05) { // || out.norm() >= 0.3) {
            out_yaw = 0;
        } else {
            integral_yaw += error_yaw;

            if (integral_yaw > 1.0) {
                integral_yaw = 1.0;
            }

            out_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw;

            if (out_yaw > 0.8) out_yaw = 0.8;
            if (out_yaw < -0.8) out_yaw = -0.8;
        }

        robo_utils::ChassisMotionControl out_velocity;
        out_velocity.exp_vx = out.x(); 
        out_velocity.exp_vy = -out.y();
        out_velocity.exp_wz = -out_yaw;

        pub.publish(out_velocity);
}

int main(int argc, char *argv[]) {
    ros::init(argc,argv, "position_control");
    
    ros::NodeHandle nh("");

    listener_p = std::make_shared<tf::TransformListener>();

    // 订阅FastLIO的里程计输出
    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("Odometry", 10, odometryCallback);

    // 订阅位置指令的话题
    //ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseStamped>("omni/pose_exp", 10, poseStampedCallback); 

    // 发布机器人底盘控制话题
    ros::Publisher pub_ctrl = nh.advertise<robo_utils::ChassisMotionControl>("offboardlink/chassis_motion_control_cmd", 1);  

    ros::Rate loop_rate(20);
    ros::Duration(3.0).sleep();
    std::thread(recdata).detach();

    while (ros::ok())
    {   
        control(pub_ctrl);   
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}       




/*

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "robo_utils/ChassisMotionControl.h"

#include <Eigen/Dense>

#include <memory>
#include <cmath>

std::shared_ptr<tf::TransformListener> listener_p;

// 初始化PID参数
double kp = 1.8, ki = 0.024;
// double kp = 0.5, ki = 0.005;
Eigen::Vector2d integral;

double kp_yaw = 1.4, ki_yaw = 0.0;
double integral_yaw;

// 位置 位置期望
Eigen::Vector2d position, goal_position, start_position;
Eigen::Quaterniond quaternion, goal_quaternion;

ros::Time time_start;

bool inited = false;

double getYawFromQuaternion(const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr msg_p) {
    position = Eigen::Vector2d(msg_p->pose.pose.position.x, msg_p->pose.pose.position.y);
    quaternion = Eigen::Quaterniond(msg_p->pose.pose.orientation.w, msg_p->pose.pose.orientation.x, msg_p->pose.pose.orientation.y, msg_p->pose.pose.orientation.z);
    if (!inited) {
        goal_position = position;
        goal_quaternion = quaternion;
        start_position = position;
        inited = true;
    }
}

void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr msg_p) {
    goal_position = Eigen::Vector2d(msg_p->pose.position.x, msg_p->pose.position.y);
    goal_quaternion = Eigen::Quaterniond(msg_p->pose.orientation.w, msg_p->pose.orientation.x, msg_p->pose.orientation.y, msg_p->pose.orientation.z);
    //ROS_INFO("Goal was set at position %f,%f", msg_p->pose.position.x, msg_p->pose.position.y); 
}

void control(ros::Publisher &pub) {

        //PID计算
        Eigen::Vector2d error, out;
        double error_yaw = getYawFromQuaternion(goal_quaternion) - getYawFromQuaternion(quaternion);
        double out_yaw;

        while (error_yaw > M_PI) {
            error_yaw -= 2 * M_PI;
        }

        while (error_yaw < -M_PI) {
            error_yaw += 2 * M_PI;
        }

        error = (quaternion.conjugate().toRotationMatrix() * (
                Eigen::Vector3d(goal_position.x(), goal_position.y(), 0) - Eigen::Vector3d(position.x(), position.y(), 0)
            )).head<2>();

        if (error.norm() < 0.05) {
            out = Eigen::Vector2d(0, 0);
        } else {
            integral += error;

            if (integral.norm() > 8.0) {
                integral = integral.normalized() * 8.0;
            }

            out = kp * error + ki * integral;

            if (out.norm() > 1.8) {
                out = out.normalized() * 1.8;
            }
        }

        if (fabs(error_yaw) < 0.05) { // || out.norm() >= 0.3) {
            out_yaw = 0;
        } else {
            integral_yaw += error_yaw;

            if (integral_yaw > 1.0) {
                integral_yaw = 1.0;
            }

            out_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw;

            if (out_yaw > 0.8) out_yaw = 0.8;
            if (out_yaw < -0.8) out_yaw = -0.8;
        }

        robo_utils::ChassisMotionControl out_velocity;
        out_velocity.exp_vx = out.x(); 
        out_velocity.exp_vy = -out.y();
        out_velocity.exp_wz = -out_yaw;

        pub.publish(out_velocity);
}

int main(int argc, char *argv[]) {
    ros::init(argc,argv, "position_control");
    
    ros::NodeHandle nh("");

    listener_p = std::make_shared<tf::TransformListener>();

    time_start = ros::Time::now();

    // 订阅FastLIO的里程计输出
    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("Odometry", 10, odometryCallback);

    // 订阅位置指令的话题
    ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseStamped>("/omnibot_1/omni/pose_exp", 10, poseStampedCallback); 

    // 发布机器人底盘控制话题
    ros::Publisher pub_ctrl = nh.advertise<robo_utils::ChassisMotionControl>("offboardlink/chassis_motion_control_cmd", 1);  

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        // 控制代码，以10HZ运行

        control(pub_ctrl);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}       

*/
