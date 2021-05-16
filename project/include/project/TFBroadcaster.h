#ifndef TFBroadcastPR_H
#define TFBroadcastPR_H

#include "PoseRecipient.h"
#include <tf2_ros/transform_broadcaster.h>
#include "ros/ros.h"
#include <string>

class TFBroadcastPR: public PoseRecipient { //inherits from poseRecepient
public:
  TFBroadcastPR(std::string fromFrame, std::string toFrame);

  void receivePose(geometry_msgs::Pose &pose);


protected:
     tf2_ros::TransformBroadcaster br;
     std::string _fromFrame;
     std::string _toFrame;
};

#endif