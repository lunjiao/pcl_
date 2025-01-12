#include<typedefine.hpp>


namespace icp_local{


bool operator<(const Time&t1,const Time&t2){
    return t1.nanosec<t2.nanosec;
}
bool operator>(const Time&t1,const Time&t2){
    return t1.nanosec>t2.nanosec;
}
bool operator<=(const Time&t1,const Time&t2){
    return t1.nanosec<=t2.nanosec;
}
bool operator>=(const Time&t1,const Time&t2){
    return t1.nanosec>=t2.nanosec;
}
bool operator==(const Time&t1,const Time&t2){
    return t1.nanosec==t2.nanosec;
}
double operator-(const Time&t1,const Time&t2){
    return t1.nanosec-t2.nanosec;
}

}