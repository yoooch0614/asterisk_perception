#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_info_setter");
    
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_priv_("~");
    
    while (!(ros::Time::now().toSec() > 0)) {}
    
    std::string param_camera_name_;
    std::string param_frame_id_;
    std::string param_camera_info_url_;
    std::string param_set_camera_info_service_;
    
    nh_priv_.param("camera_name", param_camera_name_, std::string("camera"));
    nh_priv_.param("frame_id", param_frame_id_, std::string("camera_frame"));
    nh_priv_.param("camera_info_url", param_camera_info_url_, std::string(""));
    nh_priv_.param("set_camera_info_service", param_set_camera_info_service_, std::string("set_camera_info"));
    
    
    // load CameraInfo from given camera_info_url
    if (param_camera_info_url_ == "") {
        ROS_INFO_STREAM("URL \"" << param_camera_info_url_ << "\" is not specified.");
        ros::shutdown();
    }
    
    camera_info_manager::CameraInfoManager camera_info_manager_(nh_priv_);
    
    camera_info_manager_.setCameraName(param_camera_name_);
    
    if (!camera_info_manager_.validateURL(param_camera_info_url_)) {
        ROS_ERROR_STREAM("URL \"" << param_camera_info_url_ << "\" is invalid.");
        ros::shutdown();
    }
    
    if (!camera_info_manager_.loadCameraInfo(param_camera_info_url_)) {
        ROS_ERROR_STREAM("URL \"" << param_camera_info_url_ << "\" contains invalid calibration data.");
        ros::shutdown();
    }
    
    
    // call SetCameraInfo service
    ros::service::waitForService(param_set_camera_info_service_);
    ros::ServiceClient srv_set_camera_info_cli_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>(param_set_camera_info_service_);
    
    sensor_msgs::SetCameraInfoRequest srv_set_camera_info_req_;
    sensor_msgs::SetCameraInfoResponse srv_set_camera_info_res_;
    
    srv_set_camera_info_req_.camera_info = camera_info_manager_.getCameraInfo();
    srv_set_camera_info_req_.camera_info.header.frame_id = param_frame_id_;
    
    if (!srv_set_camera_info_cli_.call(srv_set_camera_info_req_, srv_set_camera_info_res_) || !srv_set_camera_info_res_.success) {
        ROS_ERROR_STREAM("\"" << param_set_camera_info_service_ << "\" service call was failed.");
        ros::shutdown();
    }
    
    ROS_INFO_STREAM("\"" << param_set_camera_info_service_ << "\" service call was succeeded. (status message: " << srv_set_camera_info_res_.status_message << ")");
    
    ros::shutdown();
}
