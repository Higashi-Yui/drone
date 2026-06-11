#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <iomanip>

using namespace std;

class vision_pose
{
public:
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
    
    // 姿态结构体
    struct attitude {
        double roll, pitch, yaw;
    };
    
    attitude estimatedAttitude;
    attitude px4Attitude;
    geometry_msgs::PoseStamped px4Pose;

    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber px4_sub;
    ros::Publisher odom_pub;

    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_) : nh(nh_)
{
    // 订阅 Point-LIO 原始输出
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 10, &vision_pose::estimator_odom_cb, this);
    
    // 【恢复】订阅 PX4 实际位置，用于在终端进行实时对比
    px4_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &vision_pose::px4Pose_cb, this);

    // 发布给 MAVROS odometry 插件
    odom_pub = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
    
    // 初始化
    estimatedAttitude = {0, 0, 0};
    px4Attitude = {0, 0, 0};

    // 预留 8 行空间给刷新逻辑，防止顶替掉其他重要历史日志
    cout << "\033[32m [Vision Pose Bridge] 启动成功! 正在获取数据流... \033[0m" << endl;
    for(int i = 0; i < 8; ++i) {
        cout << endl; 
    }
}

void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4Pose = *msg;

    // 解析 PX4 姿态
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double r, p, y;
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    px4Attitude.roll  = r * 180.0 / 3.1415926;
    px4Attitude.pitch = p * 180.0 / 3.1415926;
    px4Attitude.yaw   = y * 180.0 / 3.1415926;
}

void vision_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 1. 发送给 PX4 (核心：透传时间戳进行延迟补偿)
    nav_msgs::Odometry mavros_odom = *msg;
    mavros_odom.header.stamp = msg->header.stamp; 
    mavros_odom.header.frame_id = "odom";
    mavros_odom.child_frame_id = "base_link";
    odom_pub.publish(mavros_odom);

    // 2. 解析 Point-LIO 姿态
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double r, p, y;
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    estimatedAttitude.roll  = r * 180.0 / 3.1415926;
    estimatedAttitude.pitch = p * 180.0 / 3.1415926;
    estimatedAttitude.yaw   = y * 180.0 / 3.1415926;

    // 3. 【修复】终端打印逻辑 (增加 \r 归位和 flush 防撕裂)
    cout << "\r\033[8A"; 
    cout << "\033[K\033[33m ---> 实时数据流比对 (Point-LIO vs PX4) \033[0m" << endl;
    cout << "\033[K" << "          SLAM(输入飞控)         PX4(实际解算)" << endl;
    cout << setiosflags(ios::fixed) << setprecision(4);
    cout << "\033[K" << "X (m)   : " << mavros_odom.pose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
    cout << "\033[K" << "Y (m)   : " << mavros_odom.pose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
    cout << "\033[K" << "Z (m)   : " << mavros_odom.pose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
    cout << "\033[K" << "Roll(°) : " << estimatedAttitude.roll << "\t\t" << px4Attitude.roll << endl;
    cout << "\033[K" << "Pitch(°): " << estimatedAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
    cout << "\033[K" << "Yaw(°)  : " << estimatedAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
    cout << flush; // 强制刷新缓冲区，彻底告别闪烁
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose_node");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");

    vision_pose vision(nh_, nh_private_);
    
    ros::spin(); 
    return 0;
}
