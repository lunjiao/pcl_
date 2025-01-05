#pragma once
#include<ImuBufferAndReading.hpp>
#include<typedefine.hpp>
#include<rclcpp/rclcpp.hpp>
namespace icp_local{

class ImuTracker{
public:

ImuTracker(std::shared_ptr<rclcpp::Node> node);
void imuCallback(const sensor_msgs::msg::Imu&imu);
Matrix4f getTransform();
//更新数据，数据从这里进来
void addReading(const Time &t,const ImuReadingd&reading);
//去重力
Vectord removeGravity(const Vectord &acc,const Quaterniond &imuOrientation);
//整合
void intergrateImuReading(const ImuReadingd&reading,const Time&t);
//计算重力在IMU坐标系中的分量
void computeGravityComponent(const Quaterniond& q, double& gx, double& gy, double& gz);
//获取位移
Vectord getIntegratedPosition(){return integratedPosition_;}
Eigen::Matrix3d getIntegratedOrien(){return integratedOrien_;}
Quaterniond getIntegratedQuaterniond(){return integratedQuaterniond_;}


//获取旋转
private:

Vectord integratedLinearVelocity_;//线速度
Vectord integratedPosition_;//位置
Vectord integratedAcc_;//角速度
Matrix3d integratedOrien_;//方向

Quaterniond integratedQuaterniond_;

ImuBuffer imuBuffer_;

std::shared_ptr<rclcpp::Node>node_;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_;

std::string imuTopic="imu_topic";

Time tprev_;//上一次的时间

double gravityFromZ;
};



}
