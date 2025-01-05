#pragma once
#include<typedefine.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<rclcpp/rclcpp.hpp>
#include<ImuTracker.hpp>
#include<queue>
namespace icp_local{



class RangeData{
public:
RangeData(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<ImuTracker>imuTracker):node_(node){
    cloudSubscriber_=node_->create_subscription<sensor_msgs::msg::PointCloud2>(rangeDataTopic_, 10,
      std::bind(&RangeData::cloudCallback, this,std::placeholders::_1));
    /*lidar_imu_R<< -1,     -1.22465e-16,            0,
          1.22465e-16,          -1,            0,
           0,            0,           1; */
    
}
void cloudCallback(const sensor_msgs::msg::PointCloud2 &msg){
    addRangeData(msg);
}
void addRangeData(const sensor_msgs::msg::PointCloud2 &msg){
    rangeDataNum++;
    if(rangeDataNum==5){
        rangeDataNum=0;
        isRangeDataReady=true;
        msgToIcp_=msg;
    }
}
sensor_msgs::msg::PointCloud2 getMsgToIcp(){
    return msgToIcp_;
}

void addRangeData_(const sensor_msgs::msg::PointCloud2 &msg){
    // Eigen::Matrix3d lidar_R;
    // lidar_R = lidar_imu_R * (imu_q.toRotationMatrix()) * lidar_imu_R.inverse();
    // IMU角度积分
    // Eigen::Quaterniond imu_q = Eigen::Quaterniond::Identity(); // R
    // Eigen::Vector3d angular_velocity_1;
    // angular_velocity_1 << imu_msg_v[0]->angular_velocity.x, imu_msg_v[0]->angular_velocity.y, imu_msg_v[0]->angular_velocity.z;
    // double lastest_time = imu_msg_v[0]->header.stamp.toSec();

    // for (int i = 1; i < imu_msg_v.size(); i++)
    // {
    //     double t = imu_msg_v[i]->header.stamp.toSec();
    //     double dt = t - lastest_time;
    //     lastest_time = t;
    //     Eigen::Vector3d angular_velocity_2;
    //     angular_velocity_2 << imu_msg_v[i]->angular_velocity.x, imu_msg_v[i]->angular_velocity.y, imu_msg_v[i]->angular_velocity.z;
    //     Eigen::Vector3d aver_angular_vel = (angular_velocity_1 + angular_velocity_2) / 2.0;
    //     imu_q = imu_q * Eigen::Quaterniond(1, 0.5 * aver_angular_vel(0) * dt, 0.5 * aver_angular_vel(1) * dt, 0.5 * aver_angular_vel(2) * dt);
    // }
};

bool getIsRangeDataReady(){return isRangeDataReady;}
void popReady(){isRangeDataReady=false;}
private:

Matrix3d lidar_imu_R;// lidar->IMU 外参数旋转矩阵
bool isRangeDataReady=false;
int rangeDataNum=0;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
std::shared_ptr<rclcpp::Node>node_;

sensor_msgs::msg::PointCloud2 msg_;
sensor_msgs::msg::PointCloud2 msgToIcp_;

std::string rangeDataTopic_="range_data";
};
}