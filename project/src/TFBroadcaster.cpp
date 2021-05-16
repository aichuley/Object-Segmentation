#include "project/TFBroadcastPR.h"



TFBroadcastPR::TFBroadcastPR(std::string fromFrame, std::string toFrame) : _fromFrame(fromFrame), _toFrame(toFrame){

}


void TFBroadcastPR::receivePose(geometry_msgs::Pose &pose) {
   

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = _fromFrame;
  transformStamped.child_frame_id = _toFrame;


  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;

  transformStamped.transform.rotation.x = pose.orientation.x;
  transformStamped.transform.rotation.y = pose.orientation.y;
  transformStamped.transform.rotation.z = pose.orientation.z;
  transformStamped.transform.rotation.w = pose.orientation.w;

  br.sendTransform(transformStamped);
}