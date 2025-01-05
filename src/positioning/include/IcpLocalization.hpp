#pragma once

#include<rclcpp/rclcpp.hpp>
#include<pcl/registration/icp.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>

#include<typedefine.hpp>
#include<ImuTracker.hpp>
#include<TfPublisher.hpp>
#include<RangeData.hpp>

#include<thread>
namespace icp_local{


class IcpLocalization:public rclcpp::Node{
public:
IcpLocalization(const rclcpp::NodeOptions &options);

//设置地图和icp设置
void setMap(PointCloud::Ptr cloud);
//
void initialize();
template<typename Scalar>
Eigen::Matrix<Scalar,4,4> getTransformationMatrix(const Eigen::Matrix<Scalar,3,1>&position,
    const Eigen::Quaternion<Scalar>&orientation);
//匹配shaomiao
void matchScan();
//
void icpWorker();
//
void fromMatrix(const Matrix4f&M, Vectorf*t,Quaternionf*q);
private:
std::shared_ptr<ImuTracker> imuTracker_;
std::shared_ptr<TfPublisher> tfPublisher_;
std::shared_ptr<RangeData> rangeData_;

Vectord lastPosition_;
Quaterniond lastOrientation_;

pcl::IterativeClosestPoint<Point,Point> icp_;
pcl::VoxelGrid<Point> filter_;
PointCloud::Ptr mapCloud_;
int maxIterate=20;
bool isMapSet=false;


};


}
