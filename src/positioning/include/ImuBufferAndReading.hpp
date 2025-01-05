#pragma once
#include<deque>
#include<iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<sensor_msgs/msg/imu.hpp>

#include<typedefine.hpp>

namespace icp_local{
template<typename FloatType>
class ImuReading{
public:

using Vector = Eigen::Matrix<FloatType, 3, 1>;
using Quaternion = Eigen::Quaternion<FloatType>;

ImuReading():acc_(Vector::Zero()),angVel_(Vector::Zero()),quaternion_(Quaterniond::Identity()){}
ImuReading(const Vector&lin,const Vector&ang,const Quaternion&q)
:acc_(lin),angVel_(ang),q_(q){}
//
const Vector& acceleration() const{return acc_;}
const Vector& angularVelocity() const{return angVel_;}
const Quaternion& rotation() const{return q_;}
Vector& acceleration(){return acc_;}
Vector& angularVelocity(){return angVel_;}
Quaternion& rotation(){return q_;}
//

private:
Vector acc_;//加速度
Vector angVel_;//角速度
Quaternion q_;

};


struct TimeAndImuReading{
    ImuReadingd imu_;
    Time time_;
};


//把ros的imu转成自己的imu
TimeAndImuReading fromRostoImureading(const sensor_msgs::msg::Imu &imu){
    Vectord lin{imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z};
    Vectord ang{imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z};
    Quaterniond q{imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z};
    TimeAndImuReading result;
    result.imu_=ImuReadingd(lin,ang,q);
    result.time_=imu.header.stamp;
    return result;
}
// //计算两个相邻的imu
// TimeAndImuReading interpolate(const TimeAndImuReading&start,const TimeAndImuReading&end,const Time& time){
//     if(time<start.time_||time>end.time_){
//         std::cerr<<"interpolate失败"<<std::endl;
//     }
//     const auto &s = start.imu_;
//     const auto &e = end.imu_;
//     TimeAndImuReading result;
//      //给imu的加速度，角速度,旋转赋值
//     result.imu_.acceleration() = interpolateVector(s.acceleration(), e.acceleration(), start.time_, end.time_, time);
//     result.imu_.angularVelocity() = interpolateVector(s.angularVelocity(), e.angularVelocity(), start.time_, end.time_, time);
//     result.imu_.rotation() = interpolateQuaternion(s.rotation(), e.rotation(),start.time_, end.time_, time);
//     return result;
// }
// //计算加速度和角速度
// Vectord interpolateVector(const Vectord&start,const Vectord&final,const Time&startTime,const Time &finalTime,const Time &queryTime){
//     double duration=finalTime-startTime;
//     double factor=(queryTime-startTime)/duration;
//     Vectord result=start+(final-start)*factor;
//     return result;
// }
// //计算四元数
// Quaterniond interpolateQuaternion(const Quaterniond&start,const Quaterniond&final,const Time &startTime,const Time &finalTime,const Time &queryTime){
//     double duration=finalTime-startTime;
//     double factor=(queryTime-startTime)/duration;
//     Quaterniond result=Quaterniond(start).slerp(factor,Quaterniond(final));
//     return result;
// }


class ImuBuffer{
public:
//在插入前检查时间是否正确。插入后检查是否超过最大队列并移除。
void push(const ImuReadingd&imu,const Time&time){
    if(!imuDeque.empty()){
        if(time>imuDeque.back().time_){
            throw std::runtime_error("ImuBufferAndReading::push,大于最大时间");
        }
        if(time<imuDeque.front().time_){
            throw std::runtime_error("ImuBufferAndReading::push,小于最小时间");
        }
    }
    TimeAndImuReading T;
    T.imu_=imu,T.time_=time;
    imuDeque.push_back(T);
    checkAndRemove();
};
//检查是否超过最大队列，若超过则移除
void checkAndRemove(){
     while(imuDeque.size()>maxDequeSize){
        imuDeque.pop_front();
    }
};
//要么放回与时间相等的imu，要么放回不超过他的最接近的imu
// ImuReadingd lookup(const Time&time){
//    if(!isHasTime(time)){
//         throw std::runtime_error("ImuBufferAndReading::lookup,失败");
//     }
//     if(imuDeque.size()==1){
//         return imuDeque.front().imu_;
//     }
//     //使用find_if找到第一个满足条件的对象
//     auto measurement=std::find_if(imuDeque.begin(),imuDeque.end(),
//     [&time](const TimeAndImuReading&T){
//         return time<=T.time_;
//     });
//     //找不到
//     if(!(measurement!=imuDeque.end())){throw std::runtime_error("ImuBufferAndReading::lookup,找不到");}

//     if(measurement->time_==time){
//         return measurement->imu_;
//     }
//     //通过调用 std::prev，我们可以得到 gmeasurement 之前的那个元素，即时间上最接近但不超过指定时间 time 的 TimestampedImuReading 对象
//     const auto start = std::prev(measurement);
//     return interpolate(*start,*measurement,time).imu_;
// };
//是否有该时间
bool isHasTime(const Time&time){
    if(imuDeque.empty()){
        std::cerr<<"imuDeque为空"<<std::endl;
        return false;
    }
    return time>imuDeque.front().time_&&time<imuDeque.back().time_;

};
//是否为空
bool isEmpty(){
    return imuDeque.empty();
};
//放回尺寸
int size(){
    return imuDeque.size();
};
//返回最近的TimeAndImuReading
TimeAndImuReading &latest_measurement(int n=1){
    if(isEmpty()){
        throw std::runtime_error("ImuBufferAndReading::last_measurement,队列imuDeque为空");
    }
    return *(std::prev(imuDeque.end(),n));
}
private:
std::deque<TimeAndImuReading> imuDeque;//imu的队列
int maxDequeSize=5000;
};

}
