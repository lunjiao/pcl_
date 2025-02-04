#ifndef TYPEDEFINE_HPP
#define TYPEDEFINE_HPP
//定义类型，以便简化

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/Dense>//Matrix3d::Identity()
#include<builtin_interfaces/msg/time.hpp>


namespace icp_local{

using Point=pcl::PointXYZ;
using PointCloud=pcl::PointCloud<Point>;

// template<typename FloatType>
// class ImuReading{};


using Vectord=Eigen::Vector3d;
using Vectorf=Eigen::Vector3f;
using Quaterniond=Eigen::Quaternion<double>;
using Quaternionf=Eigen::Quaternion<float>;
using Matrix3d=Eigen::Matrix3d;
using Matrix3f=Eigen::Matrix3f;
using Matrix4d=Eigen::Matrix4d;
using Matrix4f=Eigen::Matrix4f;


using Time=builtin_interfaces::msg::Time;
bool operator<(const Time&t1,const Time&t2);
bool operator>(const Time&t1,const Time&t2);
bool operator<=(const Time&t1,const Time&t2);
bool operator>=(const Time&t1,const Time&t2);
bool operator==(const Time&t1,const Time&t2);
double operator-(const Time&t1,const Time&t2);

}

#endif