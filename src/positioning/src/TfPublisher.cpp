#include<TfPublisher.hpp>


namespace icp_local{

void TfPublisher::publishMap(){
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*mapCloud_,msg);
  msg.header.frame_id="map";
  msg.header.stamp=rclcpp::Clock().now();
  mapPub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(),"66");
}

void TfPublisher::setGrid(){
  grid_.header.frame_id=fatherFrame_;
  grid_.header.stamp=node_->now();
  grid_.info.height=10000;
  grid_.info.width=10000;
  grid_.data.resize(grid_.info.height*grid_.info.width,-1);//-1未知
  grid_.info.resolution=0.01;
  grid_.info.origin.position.x=0.0;
  grid_.info.origin.position.y=0.0;
  grid_.info.origin.position.z=0.0;
}
using namespace std::chrono_literals;
void TfPublisher::setMap(const PointCloud::Ptr cloud){
  PointCloud::Ptr cloud_1(new PointCloud());
  *cloud_1=*cloud;
  filter_->setInputCloud(cloud_1);
  filter_->setFilterFieldName("z");
  filter_->setFilterLimits(-0.5,2.0);
  filter_->filter(*mapCloud_);
  timeBase_=node_->create_wall_timer(1s,std::bind(&TfPublisher::publishMap,this));
}
void TfPublisher::setEigen(){
  normal={0.0,0.0,1.0};
  normalHat=normal.normalized();
  p0={0.0,0.0,0.0};
  u={1.0,1.0,1.0};
  u = u - (u.dot(normalHat) * normalHat); // 确保u与法向量正交
  u.normalize();
  v = normalHat.cross(u); // 计算另一个正交向量
  v.normalize();
}
void TfPublisher::mapToGrid(){
  for(const auto &point:*mapCloud_){
    Vectorf p(point.x,point.y,point.z);
    //float d = abs(normalHat.dot(p)-normalHat.dot(p0));
    float x_uv=u.dot(p-p0);
    float y_uv=v.dot(p-p0);
    //
    int idx_1=(x_uv+10.0)*100;
    int idx_2=((y_uv+10.0)*100);
    grid_.data[idx_2*1000+idx_1]=1;
  }
}


geometry_msgs::msg::TransformStamped TfPublisher::toRos(const Vectorf &translate,const Quaternionf &q,const Time &time) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = time;
  transformStamped.header.frame_id = fatherFrame_;
  transformStamped.child_frame_id = childFrame_;
  transformStamped.transform.translation.x = translate.x();
  transformStamped.transform.translation.y = translate.y();
  transformStamped.transform.translation.z = translate.z();
  transformStamped.transform.rotation.w =q.w();
  transformStamped.transform.rotation.x =q.x();
  transformStamped.transform.rotation.y =q.y();
  transformStamped.transform.rotation.z =q.z();
  return transformStamped;
}
nav_msgs::msg::Odometry TfPublisher::toOdometry(const Vectorf &translate,const Quaternionf &q,const Time &time){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id=fatherFrame_;
  odom.header.stamp=time;
  odom.child_frame_id=childFrame_;
  odom.pose.pose.position.x=translate.x();
  odom.pose.pose.position.y=translate.y();
  odom.pose.pose.position.z=translate.z();
  odom.pose.pose.orientation.x=q.x();
  odom.pose.pose.orientation.y=q.y();
  odom.pose.pose.orientation.z=q.z();
  odom.pose.pose.orientation.w=q.w();
  return odom;
}
void TfPublisher::publishTf(const Time &time){
  geometry_msgs::msg::TransformStamped msg=toRos(realyPosition_,realyQuaterniond_,time);
  nav_msgs::msg::Odometry msg_odom=toOdometry(realyPosition_,realyQuaterniond_,time);
  odometryPub_->publish(msg_odom);
  tfBroadcaster_->sendTransform(msg);
  mapToGrid();
  grid_.header.stamp=node_->now();
  gridPub_->publish(grid_);
}


}