#include<ImuTracker.hpp>


namespace icp_local{

ImuTracker::ImuTracker(std::shared_ptr<rclcpp::Node> node):node_(node){
    imuSubscriber_=node_->create_subscription<sensor_msgs::msg::Imu>(imuTopic, 10,
      std::bind(&ImuTracker::imuCallback, this,std::placeholders::_1));
    tprev_.nanosec=0;
    Vectord zero(0.0,0.0,0.0);
    integratedPosition_=zero;
    integratedOrien_=Eigen::Matrix3d::Identity();
    integratedLinearVelocity_=zero;
    integratedAcc_=zero;

}
TimeAndImuReading fromRosToImuReading(const sensor_msgs::msg::Imu&imu){
    Vectord lin{imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z};
    Vectord ang{imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z};
    Quaterniond q{imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z};
    TimeAndImuReading result;
    result.imu_=ImuReadingd(lin,ang,q);
    result.time_=imu.header.stamp;
    return result;
}

void ImuTracker::imuCallback(const sensor_msgs::msg::Imu&imu){
    //先转类型
    TimeAndImuReading imureading=fromRosToImuReading(imu);
    addReading(imureading.time_,imureading.imu_);
}
void ImuTracker::addReading(const Time &t,ImuReadingd&reading){
    //第一次
    if(tprev_.nanosec==0){
        tprev_=t;
        Vectord coeffs=reading.acceleration();
        gravityFromZ=coeffs[2];//z
    }
    if(t<tprev_){
        throw std::runtime_error("ImuTracker::addReading,时间错误");
    }
    //去重力，z轴摆正
    reading.acceleration()=removeGravity(reading.acceleration(),reading.rotation());
    imuBuffer_.push(reading,t);
    //updateOrientationFromLinearAcceleration();
    intergrateImuReading(reading,t);
}
void ImuTracker::computeGravityComponent(const Quaterniond& q, double& gx, double& gy, double& gz) {
    double gx_w = 2 * (q.x() * q.z() - q.w() * q.y());
    double gy_w = 2 * (q.w() * q.x() + q.y() * q.z());
    double gz_w = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();
    gx = gx_w * gravityFromZ;
    gy = gy_w * gravityFromZ;
    gz = gz_w * gravityFromZ;
}
Vectord ImuTracker::removeGravity(const Vectord &acc,const Quaterniond &q){
    double gx,gy,gz;
    computeGravityComponent(q,gx,gy,gz);
    Vectord acc_new={gx,gy,gz};
    return -(acc - acc_new);//负号是为了摆正z轴
}

void ImuTracker::intergrateImuReading(const ImuReadingd&reading,const Time&t){
    double dt = t - tprev_;
    //角速度处理
    Eigen::Matrix3d B; // 角速度 * 时间 = 角度（表示为反对称矩阵）
    auto msg=reading.angularVelocity();
    B << 0, -msg.z() * dt, msg.y() * dt, 
    msg.z() * dt, 0,-msg.x() * dt,
    -msg.y() * dt,msg.x() * dt, 0;
    double sigma = sqrt(pow(msg.x(), 2) + pow(msg.y(), 2) + pow(msg.z(), 2)) * dt;//角度
    integratedOrien_*= (Eigen::Matrix3d::Identity() + (sin(sigma) / sigma) * B - ((1 - cos(sigma)) / pow(sigma, 2)) * B * B);
    //位置
    Vectord acc_i=reading.acceleration();//这里应该去重力的
    Vectord acc_w = integratedOrien_ * acc_i;
    integratedLinearVelocity_+=dt*acc_w;
    integratedPosition_+=dt*integratedLinearVelocity_;
    //q
    integratedQuaterniond_.x()=(integratedOrien_(2, 1) - integratedOrien_(1, 2)) / 4;
    integratedQuaterniond_.y()=(integratedOrien_(0, 2) - integratedOrien_(2, 0)) / 4;
    integratedQuaterniond_.z()=(integratedOrien_(1, 0) - integratedOrien_(0, 1)) / 4;
    integratedQuaterniond_.x()=sqrt(1 + integratedOrien_(0, 0) + integratedOrien_(1, 1) + integratedOrien_(2, 2)) / 2;
    tprev_ = t;//更新 tPrev_。
}
Matrix4f ImuTracker::getTransform(){
    Matrix3f R;
    float w=integratedQuaterniond_.w();
    float x=integratedQuaterniond_.x();
    float y=integratedQuaterniond_.y();
    float z=integratedQuaterniond_.z();
    R << w*w + x*x - y*y - z*z, 2*(x*y - w*z), 2*(x*z + w*y),
         2*(x*y + w*z), w*w - x*x + y*y - z*z, 2*(y*z - w*x),
         2*(x*z - w*y), 2*(y*z + w*x), w*w - x*x - y*y + z*z;
    Matrix4f M=Matrix4f::Identity();
    M.block<3, 3>(0, 0) = R;
    M.block<3, 1>(0, 3) = integratedPosition_.cast<float>();
    return M;
}

}
