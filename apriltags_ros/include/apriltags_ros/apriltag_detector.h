#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <robostate_comm_msgs/StateMsg.h>
#include <robostate_comm_msgs/States.h>
#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>

namespace apriltags_ros{


class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  double size(){return size_;}
  int id(){return id_;} 
  std::string& frame_name(){return frame_name_;} 
 private:
  int id_;
  double size_;
  std::string frame_name_;
};


class AprilTagDetector{
 public:
  AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~AprilTagDetector();
 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  void stateCb(const robostate_comm_msgs::StateMsgConstPtr& state_msg_);

  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);
  float updateX(float input)
  {
    static ros::Time time_of_last_cycle_z_(0);
    ros::Duration dt = ros::Time::now() - time_of_last_cycle_z_;
    time_of_last_cycle_z_ = ros::Time::now();
    float b = 2 * float(M_PI) * getFCutX() * dt.toSec();
    float a = b / (1 + b);
    setStateX(a * input + (1 - a)*getStateX());
    return getStateX();
  }
  float updateY(float input)
  {
    static ros::Time time_of_last_cycle_z_(0);
    ros::Duration dt = ros::Time::now() - time_of_last_cycle_z_;
    time_of_last_cycle_z_ = ros::Time::now();
    float b = 2 * float(M_PI) * getFCutY() * dt.toSec();
    float a = b / (1 + b);
    setStateY(a * input + (1 - a)*getStateY());
    return getStateY();
  }
  float updateZ(float input)
  {
    static ros::Time time_of_last_cycle_z_(0);
    ros::Duration dt = ros::Time::now() - time_of_last_cycle_z_;
    time_of_last_cycle_z_ = ros::Time::now();
    float b = 2 * float(M_PI) * getFCutZ() * dt.toSec();
    float a = b / (1 + b);
    setStateZ(a * input + (1 - a)*getStateZ());
    return getStateZ();
  }
  float getStateX() { return stateX_; }
  float getFCutX() { return fcutX_; }
  void setStateX(float state) { stateX_ = state; }
  float getStateY() { return stateY_; }
  float getFCutY() { return fcutY_; }
  void setStateY(float state) { stateY_ = state; }
  float getStateZ() { return stateZ_; }
  float getFCutZ() { return fcutZ_; }
  void setStateZ(float state) { stateZ_ = state; }

 private:
  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber state_subs_;
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
  bool inverse_tf;
  bool projected_optics_;
  float stateX_;
  float fcutX_;
  float stateY_;
  float fcutY_;
  float stateZ_;
  float fcutZ_;
  bool state_set_;
  double cam_fx_;
  double cam_fy_;
  double cam_px_;
  double cam_py_;
  double fcu_pos_x_;
  double fcu_pos_y_;
  double fcu_pos_z_;
  double fcu_pos_roll_;
  double fcu_pos_pitch_;
  double fcu_pos_yaw_;
  bool use_cam_info_topic_;
  bool use_lp_filter_;
  bool get_yaw_from_tags_;
  RoboState::States drone_state_;
};



}


#endif
