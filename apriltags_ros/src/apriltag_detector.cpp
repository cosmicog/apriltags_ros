#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/MetaPose.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include <iostream>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

#define _USE_MATH_DEFINES
#include <math.h>

namespace apriltags_ros
{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh)
{
    XmlRpc::XmlRpcValue april_tag_descriptions;
    string camera_name;
    
    ROS_INFO("RoboState Edited Version: 1.0");

    if (!pnh.getParam("camera_name", camera_name))
    {
        ROS_WARN("No camera name specified");    
    }


    if (!pnh.getParam("tag_descriptions", april_tag_descriptions))
    {
        ROS_WARN("No april tags specified");
    }
    else
    {
        try
        {
            descriptions_ = parse_tag_descriptions(april_tag_descriptions);
        }
        catch (XmlRpc::XmlRpcException e)
        {
            ROS_ERROR_STREAM("Error loading tag descriptions: " << e.getMessage());
        }
    }

    if (!pnh.getParam("sensor_frame_id", sensor_frame_id_))
    {
        sensor_frame_id_ = "";
    }

    std::string tag_family;
    pnh.param<std::string>("tag_family", tag_family, "36h11");

    pnh.param<bool>("projected_optics", projected_optics_, false);
    pnh.param<bool>("use_cam_info_topic", use_cam_info_topic_, false);
    pnh.param<bool>("use_lp_filter", use_lp_filter_, false);
    pnh.param<bool>("get_yaw_from_tags", get_yaw_from_tags_, false);
    pnh.param<double>("cam_fx", cam_fx_,0);
    pnh.param<double>("cam_fy", cam_fy_,0);
    pnh.param<double>("cam_px", cam_px_,0);
    pnh.param<double>("cam_py", cam_py_,0);
    pnh.param<double>("fcu_pos_x", fcu_pos_x_,0);
    pnh.param<double>("fcu_pos_y", fcu_pos_y_,0);
    pnh.param<double>("fcu_pos_z", fcu_pos_z_,0);
    pnh.param<double>("fcu_pos_roll",fcu_pos_roll_,0);
    pnh.param<double>("fcu_pos_pitch",fcu_pos_pitch_,0);
    pnh.param<double>("fcu_pos_yaw",fcu_pos_yaw_,0);
    ROS_INFO("using cam info topic: %d", use_cam_info_topic_);
    ROS_INFO("using low pass filter: %d", use_lp_filter_);
    ROS_INFO("get yaw from tags: %d", get_yaw_from_tags_);
    ROS_INFO("cam_fx: %lf", cam_fx_);
    ROS_INFO("cam_fy: %lf", cam_fy_);
    ROS_INFO("cam_px: %lf", cam_px_);
    ROS_INFO("cam_py: %lf", cam_py_);
    ROS_INFO("fcu_pos_x: %lf", fcu_pos_x_);
    ROS_INFO("fcu_pos_y: %lf", fcu_pos_y_);
    ROS_INFO("fcu_pos_z: %lf", fcu_pos_z_);
    ROS_INFO("fcu_pos_roll: %lf",fcu_pos_roll_);
    ROS_INFO("fcu_pos_pitch: %lf",fcu_pos_pitch_);
    ROS_INFO("fcu_pos_yaw: %lf",fcu_pos_yaw_);


    const AprilTags::TagCodes* tag_codes;
    if (tag_family == "16h5")
    {
        tag_codes = &AprilTags::tagCodes16h5;
    }
    else if (tag_family == "25h7")
    {
        tag_codes = &AprilTags::tagCodes25h7;
    }
    else if (tag_family == "25h9")
    {
        tag_codes = &AprilTags::tagCodes25h9;
    }
    else if (tag_family == "36h9")
    {
        tag_codes = &AprilTags::tagCodes36h9;
    }
    else if (tag_family == "36h11")
    {
        tag_codes = &AprilTags::tagCodes36h11;
    }
    else
    {
        ROS_WARN("Invalid tag family specified; defaulting to 36h11");
        tag_codes = &AprilTags::tagCodes36h11;
    }

    tag_detector_ = boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
    state_subs_ = nh.subscribe("/State", 10, &AprilTagDetector::stateCb, this);
    image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
    image_pub_ = it_.advertise("/tag_detections_image", 1);
    detections_pub_ = nh.advertise<AprilTagDetectionArray>("/tag_detections", 1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("/tag_detections_pose", 1);
    this->fcutX_ = 2;
    this->fcutY_ = 2;
    this->fcutZ_ = 2;
    this->state_set_ = false;
}
AprilTagDetector::~AprilTagDetector()
{
    image_sub_.shutdown();
}

void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{

    if(this->drone_state_ != RoboState::States::DOCKING)
      return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    std::vector<AprilTags::TagDetection>  detections = tag_detector_->extractTags(gray);
    ROS_DEBUG("%d tag detected", (int)detections.size());

    double fx=cam_fx_;
    double fy=cam_fy_;
    double px=cam_px_;
    double py=cam_py_;

    if(use_cam_info_topic_)
    {
      if (projected_optics_)
      {
          // use projected focal length and principal point
          // these are the correct values
          fx = cam_info->P[0];
          fy = cam_info->P[5];
          px = cam_info->P[2];
          py = cam_info->P[6];
      }
      else
      {
          // use camera intrinsic focal length and principal point
          // for backwards compatability
          fx = cam_info->K[0];
          fy = cam_info->K[4];
          px = cam_info->K[2];
          py = cam_info->K[5];
      }
    }


    if (!sensor_frame_id_.empty())
        cv_ptr->header.frame_id = sensor_frame_id_;

    AprilTagDetectionArray tag_detection_array;
    geometry_msgs::PoseArray tag_pose_array;
    tag_pose_array.header = cv_ptr->header;
    geometry_msgs::PoseStamped camera_position_in_tag_frame;
    auto sent_ = false;
    BOOST_FOREACH(AprilTags::TagDetection detection, detections)
    {

        std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
        if (description_itr == descriptions_.end())
        {
            ROS_WARN_THROTTLE(10.0, "Found tag: %d,z but no description was found for it", detection.id);
            continue;
        }
        AprilTagDescription description = description_itr->second;
        double tag_size = description.size();

        detection.draw(cv_ptr->image);


        Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);

        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);
        tf::Matrix3x3 rt_m_;
        if(get_yaw_from_tags_)
        {
          tf::Quaternion quat_(rot_quaternion.x(),rot_quaternion.y(),rot_quaternion.z(),rot_quaternion.w());
          auto yaw_ = tf::getYaw(quat_);
          rt_m_.setRPY(3.1416,0,yaw_);
        } else {
          rt_m_.setRPY(3.1416,0,1.5708);
        }
        tf::Vector3 vec_tf_(transform(0, 3), transform(1, 3), transform(2, 3));
        tf::Vector3 vec_tf_inv_ = rt_m_*-vec_tf_;
        tf::Quaternion rot_quaternion_tf_;
        rt_m_.getRotation(rot_quaternion_tf_);


        geometry_msgs::PoseStamped tag_pose;
        tag_pose.pose.position.x = vec_tf_inv_.x();
        tag_pose.pose.position.y = vec_tf_inv_.y();
        tag_pose.pose.position.z = vec_tf_inv_.z();
        tag_pose.pose.orientation.x = rot_quaternion_tf_.x();
        tag_pose.pose.orientation.y = rot_quaternion_tf_.y();
        tag_pose.pose.orientation.z = rot_quaternion_tf_.z();
        tag_pose.pose.orientation.w = rot_quaternion_tf_.w();
        tag_pose.header = cv_ptr->header;  

        AprilTagDetection tag_detection;
        tag_detection.pose = tag_pose;
        tag_detection.id = detection.id;
        tag_detection.size = tag_size;
        tag_detection_array.detections.push_back(tag_detection);
        tag_pose_array.poses.push_back(tag_pose.pose);

        if(!sent_ && ( (fabs(tag_pose.pose.position.z) > 2.0 && detection.id == 2)
                     || (fabs(tag_pose.pose.position.z) <= 2.0 && (detection.id != 2))))
        {
          tf::Stamped<tf::Transform> tag_transform;
          tf::poseStampedMsgToTF(tag_pose, tag_transform);
          tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, description.frame_name(), tag_transform.frame_id_));
          tf::Transform fcu_transform_;
          fcu_transform_.setOrigin(tf::Vector3(fcu_pos_x_, fcu_pos_y_,  fcu_pos_z_));
          tf::Quaternion fcu_q_;
          fcu_q_.setRPY(fcu_pos_roll_,fcu_pos_pitch_, fcu_pos_yaw_);
          fcu_transform_.setRotation(fcu_q_);
          tf_pub_.sendTransform(tf::StampedTransform(fcu_transform_, tag_transform.stamp_, cv_ptr->header.frame_id, "fcu_vision"));
          sent_ = true;
        }      
    }
    detections_pub_.publish(tag_detection_array);
    pose_pub_.publish(tag_pose_array);
    image_pub_.publish(cv_ptr->toImageMsg());
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions)
{
    std::map<int, AprilTagDescription> descriptions;
    ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < tag_descriptions.size(); ++i)
    {
        XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
        ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        int id = (int)tag_description["id"];
        double size = (double)tag_description["size"];

        std::string frame_name;
        if (tag_description.hasMember("frame_id"))
        {
            ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
            frame_name = (std::string)tag_description["frame_id"];
        }
        else
        {
            std::stringstream frame_name_stream;
            frame_name_stream << "tag_" << id;
            frame_name = frame_name_stream.str();
        }
        AprilTagDescription description(id, size, frame_name);
        ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name);
        descriptions.insert(std::make_pair(id, description));
    }
    return descriptions;
}

void AprilTagDetector::stateCb(const robostate_comm_msgs::StateMsgConstPtr& state_msg_)
{
   this->drone_state_ = static_cast<RoboState::States>(state_msg_->state);
}

}
