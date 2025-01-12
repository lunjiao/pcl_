#include<IcpLocalization.hpp>
#include<pcl/io/pcd_io.h>

using namespace icp_local;





int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    PointCloud::Ptr mapCloud(new PointCloud());
    auto node=std::make_shared<IcpLocalization>(rclcpp::NodeOptions{});
    //RCLCPP_INFO(node->get_logger(),"66");
    std::string filename="/home/zlunj/dingwei/src/positioning/cloud_out.pcd";
    pcl::io::loadPCDFile(filename,*mapCloud);
    node->initialize();
    node->setMap(mapCloud);
    rclcpp::spin(node);
    rclcpp::shutdown();
}
