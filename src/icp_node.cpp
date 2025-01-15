#include <memory>  
#include <iostream>
#include<pcl/io/pcd_io.h>  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/registration/icp.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"  

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;  

class PointCloudICP : public rclcpp::Node {  
public:  
    PointCloudICP()  
        : Node("pointcloud_icp")  
    {  
        // 订阅局部点云  
        local_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  
            "local_pointcloud", 10, std::bind(&PointCloudICP::local_callback, this, std::placeholders::_1));  

        // 发布配准后的点云  
        transformed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_pointcloud", 10);  

        // 创建一个全局点云供 ICP 使用，这里需要加载或存储全局点云  
        global_cloud_ = loadGlobalPointCloud(); // 自定义函数加载全局点云  
    }  

private:
    std::string sub_name;
    std::string cloud_path="/home/zlunj/dingwei1/position/cloud_out.pcd";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_sub_;  
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pub_;  
    PointCloud::Ptr global_cloud_;
    int point_count=0;
    int icp_num=20;  

    void local_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        point_count++;
        if(point_count<40){
            return;
        }
        else{
            point_count=0;
        }  
        PointCloud::Ptr local_cloud(new PointCloud);  
        pcl::fromROSMsg(*msg, *local_cloud);  

        // 进行ICP配准  
        Eigen::Matrix4f transformation = performICP(local_cloud, global_cloud_);  

        // 转换局部点云  
        PointCloud::Ptr transformed_cloud(new PointCloud);  
        pcl::transformPointCloud(*local_cloud, *transformed_cloud, transformation);  

        // 发布配准后的点云  
        sensor_msgs::msg::PointCloud2 output_msg;  
        pcl::toROSMsg(*transformed_cloud, output_msg);  
        output_msg.header.frame_id = msg->header.frame_id; // 设置框架ID  
        output_msg.header.stamp = this->now();  
        transformed_pub_->publish(output_msg);  

        RCLCPP_INFO(this->get_logger(), "ICP transformation applied.");  
    }  

    Eigen::Matrix4f performICP(PointCloud::Ptr local_cloud, PointCloud::Ptr global_cloud) {  
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
        icp.setInputSource(local_cloud);  
        icp.setInputTarget(global_cloud);  
        icp.setMaximumIterations(icp_num);
        PointCloud final_cloud;  
        icp.align(final_cloud);  // 执行配准  

        if (icp.hasConverged()) {  
            return icp.getFinalTransformation();  // 获取最终变换矩阵  
        } else {  
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");  
            return Eigen::Matrix4f::Identity();  // 如果未收敛，返回单位矩阵  
        }  
    }  

    PointCloud::Ptr loadGlobalPointCloud() {  
        // 加载全局点云数据的自定义函数  
        // 这里可以实现读取点云文件，或者从某个话题中订阅  
        PointCloud::Ptr cloud(new PointCloud);
        pcl::io::loadPCDFile(cloud_path,*cloud);

        // 示例: cloud->width = ... ; cloud->height = ...   
        // cloud->points.push_back(...);  

        return cloud;  
    }  
};  

int main(int argc, char * argv[]) {  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<PointCloudICP>());  
    rclcpp::shutdown();  
    return 0;  
}