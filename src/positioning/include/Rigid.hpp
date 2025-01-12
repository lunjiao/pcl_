#ifndef RIGID_HPP
#define RIGID_HPP


#include<Eigen/Core>
#include<Eigen/Geometry>

namespace icp_local{
    
template<typename FloatType>
class Rigid{
public:
using Vector=Eigen::Vector<FloatType,3>;
using Quaternion=Eigen::Quaternion<FloatType>;


Rigid():translate_(Vector::Zero()),rotation_(Quaternion::Identity()){}
Rigid(const Vector &translate,const Quaternion&rotation)
:translate_(translate),rotation_(rotation){}

Vector &translation(){return translate_;}
const Vector &translation()const{return translate_;}

Quaternion &rotation(){return rotation_;}
const Quaternion &rotation()const{return rotation_;}


private:
Vector translate_;
Quaternion rotation_;

};

//重载乘法
template <typename FloatType>
Rigid<FloatType> operator*(const Rigid<FloatType> &left,const Rigid<FloatType> &right){
    return Rigid<FloatType>(
        left.rotation()*right.translation()+left.translation(),
        (left.rotation()*right.rotation()).normalized());
}
//左值是否大于右值
template<typename T>
bool assert_1(const T&t_1,const T&t_2){
    if(t_1<t_2){
        throw std::runtime_error("assert_1出错");
    }
}
using Rigid3d=Rigid<double>;

}

#endif