#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <atomic>
#include "robo_utils/ChassisMotionControl.h"
#include <Eigen/Dense>
#include <memory>
#include <cmath>


#define PORT 12345  //Socket监听端口

/*****控制对齐方式，保证数据传输正确*****/
#pragma pack(push, 1)   
struct PosStruct {      //发送数据包格式
    float x;
    float y;
    float qw;
    float qx;
    float qy;
    float qz;
    void print() const {
        //std::cout << "x=" << x << ", y=" << y << ", qw=" << qw << ", qx=" << x << ", qy=" << qy << ", qz=" << qz;
    }
};
#pragma pack(pop)

// 定义一个结构体来存储位置和角度数据
struct go_RobotState {
    double x;
    double y;
    geometry_msgs::Quaternion orientation;
};

// 函数声明
go_RobotState eight_RobotState(double startTime, double currentTime, double speed,double radius);
go_RobotState cercle_RobotState(double startTime, double currentTime, double speed, double radius);
go_RobotState line_RobotState(double startTime, double currentTime, double speed);
go_RobotState S_RobotState(double startTime, double currentTime, double speed, double amplitude, double frequency);
go_RobotState sin_RobotState(double startTime, double currentTime, double speed, double amplitude, double frequency);
double speed = 1.0; // 控制速度参数
double radius= 0.5; // 控制半径参数
double amplitude=0.5; // S形状曲线幅度
double frequency=M_PI;// 频率参数
//以上为示例函数路线相关变量



double getYawFromQuaternion(const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
Eigen::Quaterniond getQuaternionFromYaw(double angle) {
    return Eigen::Quaterniond(0,0,sin(angle/2),cos(angle/2));
}
bool SendData(int client_id,Eigen::Vector2d pos,Eigen::Quaterniond q);


//三轮机器人类    私有：ID，坐标，四元数，偏航角  公有：发布/走X/走Y/走XY/旋转/......
class Omnibot{
    private:
        int ID;
        Eigen::Vector2d Pos;
        Eigen::Quaterniond Q;
        double Yaw;
    public:
        Omnibot(int id) : Pos(0, 0), Q(1, 0, 0, 0) {ID=id;}
        void Operate() {SendData(ID,Pos,Q);}
        void StepX(float dx){Pos.x()+=dx;}
        void StepY(float dy){Pos.y()+=dy;}
        void StepXY(float dx,float dy){Pos.x()+=dx;Pos.y()+=dy;}
        void SetX(float x){Pos.x()=x;}
        void SetY(float x){Pos.y()=x;}
        void SetXY(float x,float y){Pos.x()=x;Pos.y()=y;}
        void Rotate(float Angle)
        {
            Yaw=getYawFromQuaternion(Q);
            Yaw+=Angle;
            Q=getQuaternionFromYaw(Yaw);
        }
        void SetAngle(float Angle)
        {
            Yaw=Angle;
            Q=getQuaternionFromYaw(Yaw);
        }
};


Omnibot Bot1(1),Bot2(2),Bot3(3); 
ros::Publisher hostpospub;

class ClientManager {
    private:
        std::mutex mtx_;
        std::atomic<int> next_id_{1}; //客户端开始编号的数值，例如2号车为从机，那么可以从2开始给客户端编号
        std::unordered_map<int, int> id_to_fd_;  // client_id -> fd
        std::unordered_map<int, int> fd_to_id_;  // fd -> client_id
    
    public:
        int add_client(int fd) {
            std::lock_guard<std::mutex> lock(mtx_);
            int id = next_id_++;
            id_to_fd_[id] = fd;
            fd_to_id_[fd] = id;
            return id;
        }
    
        void remove_client(int fd) {
            std::lock_guard<std::mutex> lock(mtx_);
            if (auto it = fd_to_id_.find(fd); it != fd_to_id_.end()) {
                id_to_fd_.erase(it->second);
                fd_to_id_.erase(it);
            }
        }
    
        int get_fd(int client_id) {
            std::lock_guard<std::mutex> lock(mtx_);
            if (auto it = id_to_fd_.find(client_id); it != id_to_fd_.end()) {
                return it->second;
            }
            return -1;
        }
    
        void list_clients() {
            std::lock_guard<std::mutex> lock(mtx_);
            std::cout << "\nConnected clients (" << id_to_fd_.size() << "):\n";
            for (const auto& [id, fd] : id_to_fd_) {
                std::cout << "Client ID: " << id << " (FD: " << fd << ")\n";
            }
        }
};
    
ClientManager client_manager;

//>>>客户端处理线程<<<    
void handle_client(int client_fd, sockaddr_in client_addr) {
    // 注册客户端
    int client_id = client_manager.add_client(client_fd);
    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    
    std::cout << "[+] Client " << client_id << " connected from "
                  << client_ip << ":" << ntohs(client_addr.sin_port) << std::endl;
    
    // 接收循环
    while (true) {
        PosStruct response;
        int bytes = recv(client_fd, &response, sizeof(PosStruct), 0);
        
        if (bytes <= 0) {
            if (bytes == 0) {
                std::cout << "[-] Client " << client_id << " disconnected\n";
            } else {
                perror("recv error");
            }
            break;
        }

        // 显示响应
        std::cout << "Response from client " << client_id << ": ";
        response.print();
        std::cout << std::endl;
    }
    
    // 清理资源
    close(client_fd);
    client_manager.remove_client(client_fd);
}
    
//  >>>>手动发布数据线程<<<<
void send_pos()
{
    std::cout<<"发布坐标线程已启动"<<std::endl;
    while(true)
    {
        int client_id;
        float x,y;
        int qw,qx,qy,qz;
        std::cout<<"please text target client id:";
        std::cin>>client_id;
        int target_fd = client_manager.get_fd(client_id);
        std::cout<<"please text x y qw qx qy qz:";
        std::cin>>x>>y>>qw>>qx>>qy>>qz;
        PosStruct data{x,y,qw,qx,qy,qz};
        if (target_fd != -1) 
        {
            if (send(target_fd, &data, sizeof(data), 0) == -1)
            {
                perror("send failed");
            }
            else 
            {
                std::cout << "Sent data to client " << client_id << ": ";
                data.print();
                std::cout << std::endl;
            }
        } 
        else 
        {
            std::cout << "Invalid client ID\n";
        }
    }
}

//>>>监听并自动添加客户端线程<<<
bool addclient()
{
         // 创建监听socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("socket creation failed");
        return 1;
    }

    // 设置地址重用
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定地址
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr))) {
        perror("bind failed");
        close(server_fd);
        return 1;
    }

    // 开始监听
    if (listen(server_fd, SOMAXCONN)) {
        perror("listen failed");
        close(server_fd);
        return 1;
    }
        while(1)
        {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
            
            if (client_fd == -1) {
                perror("accept failed");
                continue;
            }

            // 为每个客户端创建线程
            std::thread(handle_client, client_fd, client_addr).detach();
        }
        close(server_fd);
}

bool SendData(int client_id,Eigen::Vector2d pos,Eigen::Quaterniond q)
{
    ROS_INFO("Send ID:%d   POS:(%.2f,%.2f)  Q:(%.2f,%.2f,%.2f,%.2f)",client_id,pos.x(),pos.y(),q.w(),q.x(),q.y(),q.z());
    if(client_id==0) //0号ID指主机通过ros话题通信给自己发布坐标数据
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x=pos.x();
        pose.pose.position.x=pos.y();
        pose.pose.orientation.w=q.w();
        pose.pose.orientation.x=q.x();
        pose.pose.orientation.y=q.y();
        pose.pose.orientation.z=q.z();
        hostpospub.publish(pose);
        return true;
    }
    else if(client_id>0)
    {
        int target_fd = client_manager.get_fd(client_id);
        if (target_fd != -1) 
        {
            PosStruct data{pos.x(),pos.y(),q.w(),q.x(),q.y(),q.z()};
            if (send(target_fd, &data, sizeof(data), 0) == -1)
            {
                perror("send failed");
            }
            else 
            {
                std::cout << "Sent data to client " << client_id << ": ";
                data.print();
                std::cout << std::endl;
            }
        } 
        else 
        {
            std::cout << "Invalid client ID\n";
        }
        return true;
    }
    else
    {
        ROS_INFO("Invalid Client ID");
        return false;
    }
}

//>>>自动规划路径函数<<<
/*在此函数内编写给三个机器人布置路线，操作Omnibot类中的共有函数即可*/
void Route()
{
    double startTime = ros::Time::now().toSec();
    double currentTime = ros::Time::now().toSec();
    ROS_INFO("规划路线线程启动...10s后开始行动");
    sleep(10);
    float dt=0.1;
    //示例编队变换代码
    switch(a)
    {
        case 1:
        {
            while(ros::ok())
            {
                //"1"变"Δ"
                Bot1.StepY(-0.1);
                Bot2.StepX(0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot1.StepY(-0.1);
                Bot2.StepX(0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot1.StepY(-0.1);
                Bot2.StepX(0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot1.StepY(-0.1);
                Bot2.StepX(0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot1.StepY(-0.1);
                Bot2.StepX(0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot2.StepX(0.1);
                sleep(dt);
                Bot2.StepX(0.1);
                sleep(dt);
                Bot2.StepX(0.1);
                sleep(dt);
                Bot2.StepX(0.066);
                sleep(50*dt);
                //向前0.5m
                Bot1.StepX(0.1);
                Bot2.StepX(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepX(0.1);
                Bot2.StepX(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepX(0.1);
                Bot2.StepX(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepX(0.1);
                Bot2.StepX(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepX(0.1);
                Bot2.StepX(0.1);
                Bot3.StepX(0.1);
                sleep(50*dt);
                //"Δ"变"一"
                Bot1.StepY(0.1);
                Bot3.StepX(-0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot3.StepX(-0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot3.StepX(-0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot3.StepX(-0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot3.StepX(-0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot3.StepX(-0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot3.StepX(-0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot3.StepX(-0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot3.StepX(-0.066);
                Bot3.StepY(0.1);
                sleep(50*dt);
                //"一"变"1"
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot2.StepX(-0.066);
                Bot3.StepY(0.1);
                Bot3.StepX(0.1);
                sleep(dt);
                Bot1.StepY(0.1);
                Bot3.StepY(0.1);
                Bot3.StepX(0.066);
                sleep(50*dt);
                //向后0.5m，回到原位
                Bot1.StepX(-0.1);
                Bot2.StepX(-0.1);
                Bot3.StepX(-0.1);
                sleep(50*dt);
                Bot1.StepX(-0.1);
                Bot2.StepX(-0.1);
                Bot3.StepX(-0.1);
                sleep(50*dt);
                Bot1.StepX(-0.1);
                Bot2.StepX(-0.1);
                Bot3.StepX(-0.1);
                sleep(50*dt);
                Bot1.StepX(-0.1);
                Bot2.StepX(-0.1);
                Bot3.StepX(-0.1);
                sleep(50*dt);
                Bot1.StepX(-0.1);
                Bot2.StepX(-0.1);
                Bot3.StepX(-0.1);
                sleep(50*dt);
                //循环执行队形变换
            }
        }
        case 2:
        {
            go_RoboState stanew;
            while(ros::ok())
            {
                //可以在这自定义小车路径
                //例：8字路径
                double currentTime = ros::Time::now().toSec();
                stanew=eight_RobotState(startTime,currentTime,speed,radius);
                Bot1.SetXY(stanew.x,stanew.y);
                Bot2.SetXY(stanew.x,stanew.y);
                Bot3.SetXY(stanew.x,stanew.y);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc,argv, "robot_link");
    ros::NodeHandle nh("");
    hostpospub = nh.advertise<geometry_msgs::PoseStamped>("/omnibot_1/omni/pose_exp",10);
    std::thread(addclient).detach();  //监听客户端添加线程
    
    std::cout<<"LINK发布坐标线程已启动"<<std::endl;
    ROS_INFO("server port:%d",PORT);
    ros::Rate loop_rate(10);

    std::cout<<"演示类型："<<std::endl;
    std::cout<<"1-队形变换演示"<<std::endl;
    std::cout<<"2-8字路线演示"<<std::endl;
    std::cout<<"请输入演示内容：";
    int a;
    std::cin>>a;
    std::thread(Route).detach(); //路线规划线程


    
    //主函数内循环发布3台车的位置数据 
    while(ros::ok())  
    {
        // double currentTime = ros::Time::now().toSec();  //基于时间的位置发布
        Bot1.Operate();
        Bot2.Operate();
        Bot3.Operate();
        loop_rate.sleep();
    }

}


//基于时间的复杂曲线规划函数
go_RobotState eight_RobotState(double startTime, double currentTime, double speed,double radius) {
    double elapsedTime = currentTime - startTime;
    double angularSpeed = speed * 2 * M_PI / 10; // 10 seconds for a full cycle

    // 计算八字形轨迹
    double y = sin(angularSpeed * elapsedTime)* radius;
    double x = sin(angularSpeed * elapsedTime) * cos(angularSpeed * elapsedTime)* radius;

    // 计算机器人的朝向
    tf2::Quaternion q;
    q.setRPY(0, 0, atan2(y, x));

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    go_RobotState state = {x, y, orientation};
    return state;
}

// 计算圆形轨迹的机器人状态
go_RobotState cercle_RobotState(double startTime, double currentTime, double speed, double radius) {
    double elapsedTime = currentTime - startTime;

    // 计算机器人的位置（沿着圆周运动）
    double x = radius * cos(speed * elapsedTime);
    double y = radius * sin(speed * elapsedTime);

    // 计算机器人朝向（切线方向）
    // 切线方向的角度是沿着圆的切线方向
    double theta = speed * elapsedTime; // 机器人朝向的角度
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    go_RobotState state = {x, y, orientation};
    return state;
}

// 计算直线形轨迹的机器人状态
go_RobotState line_RobotState(double startTime, double currentTime, double speed) {
    double elapsedTime = currentTime - startTime;
    double y = speed*elapsedTime;
    double x = 0.0;
    geometry_msgs::Quaternion orientation;
    orientation.x = 1;
    orientation.y = 0;
    orientation.z = 0;
    orientation.w = 0;

    go_RobotState state = {x, y, orientation};
    return state;
}
//走S形
go_RobotState S_RobotState(double startTime, double currentTime, double speed, double amplitude, double frequency) {
    double elapsedTime = currentTime - startTime;

    // 计算机器人的位置，沿直线 x 轴和 S 形 y 轴
    double y = speed * elapsedTime;
    double x = amplitude * sin(frequency * elapsedTime);

    // 计算机器人朝向，假设机器人朝向沿 S 形轨迹的切线方向
    // 在 x 和 y 方向上的运动速度的比率决定了朝向
    double theta = atan2(amplitude * frequency * cos(frequency * elapsedTime), speed);  // 计算切线方向的角度
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    go_RobotState state = {x, y, orientation};
    return state;
}

// 计算正弦形状轨迹的机器人状态
go_RobotState sin_RobotState(double startTime, double currentTime, double speed, double amplitude, double frequency) {
    double elapsedTime = currentTime - startTime;

    // 计算机器人的位置，沿直线 x 轴和正弦波动的 y 轴
    double x = speed * elapsedTime;
    double y = amplitude * sin(frequency * elapsedTime);

    // 计算机器人朝向，假设机器人朝向沿正弦轨迹的切线方向
    // 在 x 和 y 方向上的运动速度的比率决定了朝向
    double theta = atan2(amplitude * frequency * cos(frequency * elapsedTime), speed);  // 计算切线方向的角度
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    go_RobotState state = {x, y, orientation};
    return state;
}