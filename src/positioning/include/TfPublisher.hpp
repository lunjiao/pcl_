#ifndef TFPUBLISHER_HPP
#define TFPUBLISHER_HPP

#include<rclcpp/rclcpp.hpp>
#include<pcl_conversions/pcl_conversions.h>
#include<nav_msgs/msg/odometry.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>
#include<nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/passthrough.h>
#include<typedefine.hpp>
#include<ImuTracker.hpp>
#include<chrono>
namespace icp_local{
class TfPublisher{
public:

TfPublisher(std::shared_ptr<rclcpp::Node>node,
std::shared_ptr<ImuTracker>imuTracker):node_(node),imuTracker_(imuTracker){
    mapCloud_=std::make_shared<PointCloud>();
    filter_=std::make_shared<pcl::PassThrough<pcl::PointXYZ>>();
    tfBroadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    gridPub_=node_->create_publisher<nav_msgs::msg::OccupancyGrid>(gridTopic_,10);
    odometryPub_=node_->create_publisher<nav_msgs::msg::Odometry>(odomTopic,10);
    mapPub_=node_->create_publisher<sensor_msgs::msg::PointCloud2>(mapTopic,10);
    setGrid();
}
void publishMap();
void setEigen();
void setMap(const PointCloud::Ptr cloud);//函数名不一样
void setGrid();
void mapToGrid();
void publishTf(const Time &time);
geometry_msgs::msg::TransformStamped toRos(const Vectorf &translate,const Quaternionf &q,const Time &time);
nav_msgs::msg::Odometry toOdometry(const Vectorf &translate,const Quaternionf &q,const Time &time);

void setRealyPosition(const Vectorf&realyPosition){realyPosition_=realyPosition;}
void setRealyQuaterniond(const  Quaternionf&realyQuaterniond){realyQuaterniond_=realyQuaterniond;}
const Vectorf &getRealyPosition()const{return realyPosition_;}
const Quaternionf &getRealyQuaterniond()const{return realyQuaterniond_;}


private:
PointCloud::Ptr mapCloud_;
pcl::PassThrough<pcl::PointXYZ>::Ptr filter_;

std::shared_ptr<rclcpp::Node> node_;
std::shared_ptr<ImuTracker> imuTracker_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridPub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPub_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPub_;
rclcpp::TimerBase::SharedPtr timeBase_;
nav_msgs::msg::OccupancyGrid grid_;

Vectorf realyPosition_;//位置
Quaternionf realyQuaterniond_;
Vectorf normal;//法向量
Vectorf normalHat;//法向量的单位向量
Vectorf p0;//平面上的点
Vectorf u;
Vectorf v;

std::string fatherFrame_="map";
std::string childFrame_="body";
std::string gridTopic_="grid_Topic";
std::string odomTopic="odom_Topic";
std::string mapTopic="map_Topic";

};
}

#endif