#include<IcpLocalization.hpp>

using namespace icp_local;



int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<IcpLocalization>(rclcpp::NodeOptions{});
    
    PointCloud::Ptr mapCloud;
    node->setMap(mapCloud);
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
}