#include<IcpLocalization.hpp>



namespace icp_local{

IcpLocalization::IcpLocalization(const rclcpp::NodeOptions &options):Node("icp_localization",options){
    filter_.setLeafSize(0.05f,0.05f,0.05f);
    mapCloud_=std::make_shared<PointCloud>();
};

void IcpLocalization::initialize(){
    imuTracker_=std::make_shared<ImuTracker>(this->shared_from_this());
    tfPublisher_=std::make_shared<TfPublisher>(this->shared_from_this(),imuTracker_);
    rangeData_=std::make_shared<RangeData>(this->shared_from_this());
    std::thread icp_worke_thread(std::bind(&IcpLocalization::icpWorker,this));
    icp_worke_thread.detach();
}

void IcpLocalization::setMap(PointCloud::Ptr cloud){
    //过滤
    filter_.setInputCloud(cloud);
    filter_.filter(*mapCloud_);
    icp_.setInputTarget(mapCloud_);
    icp_.setMaximumIterations(maxIterate);
    //RCLCPP_INFO(this->get_logger(),"66");
    tfPublisher_->setMap(mapCloud_);
    isMapSet=true;
    
}
void IcpLocalization::matchScan(){
    if(!isMapSet){
        throw std::runtime_error("地图未设置!");
    }
    sensor_msgs::msg::PointCloud2 msg=rangeData_->getMsgToIcp();
    PointCloud::Ptr cloud;
    pcl::fromROSMsg(msg, *cloud);
    //icp
    Matrix4f M=imuTracker_->getTransform();
    icp_.setInputSource(cloud);
    icp_.align(*cloud,M);
    if(!icp_.hasConverged()){
        throw std::runtime_error("icp不收敛");
    }
    std::cout<<"误差:"<<icp_.getFitnessScore()<<std::endl;
    const Matrix4f& transformation = icp_.getFinalTransformation();
    Vectorf t;
    Quaternionf q;
    fromMatrix(transformation,&t,&q);//提取t和q
    tfPublisher_->setRealyPosition(t);
    tfPublisher_->setRealyQuaterniond(q);
    tfPublisher_->publishTf(msg.header.stamp);//发布
}
void IcpLocalization::icpWorker() {
  
  /*
  在这个循环中，r.sleep()将确保每次迭代之间的时间间隔大约为0.01秒（1/100秒），从而尝试保持100Hz的频率。
  如果循环中的操作花费的时间少于0.01秒，r.sleep()将使当前线程休眠以补偿；如果操作花费的时间超过0.01秒，r.sleep()可能不会休眠，或者只休眠很短的时间。
  */
  // ros::Rate r(100);
  //检查完后才走程序
    rclcpp::Rate r(100);
    while (rclcpp::ok()) {
        if (!rangeData_->getIsRangeDataReady()) {
        r.sleep();
        continue;
        }
    }
    matchScan();
    rangeData_->popReady();
    r.sleep();
}

void IcpLocalization::fromMatrix(const Matrix4f&M, Vectorf*t,Quaternionf*q){
    //提取旋转矩阵
    Matrix3f R = M.block<3,3>(0,0);
    float w = sqrt(1 + R(0,0) + R(1,1) + R(2,2)) / 2;
    float x = (R(2,1) - R(1,2)) / (4 * w);
    float y = (R(0,2) - R(2,0)) / (4 * w);
    float z = (R(1,0) - R(0,1)) / (4 * w);
    *q=Quaternionf(w, x, y, z);
    //
    *t= M.block<3,1>(0,3);
}



// template<typename Scalar>
// Eigen::Matrix<Scalar,4,4> IcpLocalization::getTransformationMatrix(
//     const Eigen::Matrix<Scalar,3,1>&position,
//     const Eigen::Quaternion<Scalar>&orientation)
// {
//     Eigen::Matrix<Scalar,-1,-1> matrix(4,4);
//     matrix.setIdentity();//归一
//     matrix.block(0, 3, 3, 1)=position;//前3行的第4列
//     matrix.block(0, 0, 3, 3)=orientation;//前3行的前3列
//     return matrix;
// }

}