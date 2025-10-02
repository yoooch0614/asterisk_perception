// c++
#include <vector>
#include <cstdlib>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

// ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

namespace asterisk
{


class PovCameraJoyController
{
private:
   // ros::NodeHandle nh_jp;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    ros::Subscriber sub_joy_;
   // ros::Publisher pub_pan_angle = nh_jp.advertise<std_msgs::String>("joy_pan_angle", 1000);

    double param_frequency_;
    std::string param_pov_camera_frame_;
    std::string param_camera_frame_;
    
    bool param_use_cross_;
    bool param_pan_reverse_;
    double param_pan_init_;
    double param_pan_velocity_;
    bool param_tilt_reverse_;
    double param_tilt_init_;
    double param_tilt_velocity_;
    bool param_roll_reverse_;
    double param_roll_init_;
    double param_roll_velocity_;
    
    tf::Vector3 translate_;
    double angle_roll_, angle_pitch_, angle_yaw_;
    double velocity_roll_, velocity_pitch_, velocity_yaw_;
    bool reset_;
    int pan_direction_;
    int tilt_direction_;
    int roll_direction_;
    
    const double RAD2DEG = 180.0 / M_PI;
    const double DEG2RAD = M_PI / 180.0;
    
    // joy - ROS Wiki
    // http://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux
    enum JoyKeymap {
        BUTTON_A = 0,
        BUTTON_B = 1,
        BUTTON_X = 2,
        BUTTON_Y = 3,
        BUTTON_LB = 4,
        BUTTON_RB = 5,
        BUTTON_START = 6,
        BUTTON_BACK  = 7,
        BUTTON_POWER = 8,
        BUTTON_LSTICK = 9,
        BUTTON_RSTICK = 10,
        BUTTON_CROSS_LEFT  = 11,
        BUTTON_CROSS_RIGHT = 12,
        BUTTON_CROSS_UP    = 13,
        BUTTON_CROSS_DOWN  = 14,
            
        AXIS_LSTICK_LR = 0,
        AXIS_LSTICK_UD = 1,
        AXIS_LTRIGGER  = 2,
        AXIS_RSTICK_LR = 3,
        AXIS_RSTICK_UD = 4,
        AXIS_RTRIGGER  = 5,
        AXIS_CROSS_LR = 6,
        AXIS_CROSS_UD = 7,
        AXIS_MAX = 8,
    };
    
    double value2wrapped(const double value, const double upper, const double lower);
    double angle2wrapped(const double angle);
    double joy2angle(const double joy_value, const double joy_max, const double joy_min, const double angle_mix, const double angle_max);
    
public:
    PovCameraJoyController(void);
    void joyCallback(const sensor_msgs::Joy &joy);
    void updatePovCameraTransform();
    void loop();
};


double PovCameraJoyController::value2wrapped(const double value, const double upper, const double lower)
{
    // if ((upper - lower) < std::numeric_limits<double>::epsilon() || upper < lower) {
    //     throw std::domain_error("given arguments: upper < lower");
    //     return value;
    // }
    
    if (value > upper) {
        return std::fmod(value - lower, upper - lower) + lower;
    }
    else if (value < lower) {
        return std::fmod(value - upper, upper - lower) + upper;
    }
    else {
        return value;
    }
}


double PovCameraJoyController::angle2wrapped(const double angle)
{
    return value2wrapped(angle, + M_PI, - M_PI);
}


double PovCameraJoyController::joy2angle(const double joy_value, const double joy_max, const double joy_min, const double angle_max, const double angle_min)
{
    return (angle_max - angle_min) / (joy_max - joy_min) * (joy_value - joy_min) + angle_min;
}


PovCameraJoyController::PovCameraJoyController(void)
    : nh_("")
    , nh_priv_("~")
{
    // wait for clock (in /use_sim_time = true)
    while (!(ros::Time::now().toSec() > 0.0)) {}
    
    // parameter initialization
    nh_priv_.param("frequency", param_frequency_, 100.0);
    nh_priv_.param("pov_camera_frame", param_pov_camera_frame_, std::string("flipped_pov_camera"));
    nh_priv_.param("camera_frame", param_camera_frame_, std::string("camera/flipped_front"));
    nh_priv_.param("use_cross", param_use_cross_, false);
    nh_priv_.param("pan/reverse", param_pan_reverse_, false);
    nh_priv_.param("pan/init", param_pan_init_, 0.0);
    nh_priv_.param("pan/velocity", param_pan_velocity_, M_PI / 5.0);
    nh_priv_.param("tilt/reverse", param_tilt_reverse_, false);
    nh_priv_.param("tilt/init", param_tilt_init_, 0.0);
    nh_priv_.param("tilt/velocity", param_tilt_velocity_, M_PI / 8.0);
    nh_priv_.param("roll/reverse", param_roll_reverse_, false);
    nh_priv_.param("roll/init", param_roll_init_, 0.0);
    nh_priv_.param("roll/velocity", param_roll_velocity_, M_PI / 8.0);
    
    if (std::abs(param_frequency_) < std::numeric_limits<double>::epsilon()) {
        ROS_WARN_STREAM("Invalid frequency parameter: " << param_frequency_);
        ROS_WARN_STREAM("Frequency is set 100.0 Hz instead." << param_frequency_);
        param_frequency_ = 100.0;
    }
    else {
        param_frequency_ = std::abs(param_frequency_);
    }
    
    angle_roll_  = param_roll_init_;
    angle_pitch_ = param_tilt_init_;
    angle_yaw_   = param_pan_init_;
    velocity_roll_  = 0.0;
    velocity_pitch_ = 0.0;
    velocity_yaw_   = 0.0;
    
    pan_direction_ = param_pan_reverse_   ? -1 : +1;
    tilt_direction_ = param_tilt_reverse_ ? +1 : -1;
    roll_direction_ = param_roll_reverse_ ? +1 : -1;
    
    sub_joy_ = nh_.subscribe("joy", 1, &PovCameraJoyController::joyCallback, this);
}


void PovCameraJoyController::joyCallback(const sensor_msgs::Joy &joy)
{


    if (joy.axes.size() < JoyKeymap::AXIS_MAX)
    {
        ROS_ERROR("Insufficient axes in Joy message.");
        return;
    }
    // Proceed to use joy.axes safely...

    if (joy.buttons.size() < JoyKeymap::BUTTON_RSTICK)
    {
        ROS_ERROR("Insufficient buttons in Joy message.");
        return;
    }

    double velocity_roll_ltrigger = std::abs(joy.axes[JoyKeymap::AXIS_LTRIGGER]) < std::numeric_limits<double>::epsilon() ? 
                                    0.0 :
                                    joy2angle(roll_direction_ * joy.axes[JoyKeymap::AXIS_LTRIGGER], 1.0, -1.0, 0.0, - std::abs(param_roll_velocity_));
    double velocity_roll_rtrigger = std::abs(joy.axes[JoyKeymap::AXIS_RTRIGGER]) < std::numeric_limits<double>::epsilon() ? 
                                    0.0 :
                                    joy2angle(roll_direction_ * joy.axes[JoyKeymap::AXIS_RTRIGGER], 1.0, -1.0, 0.0, std::abs(param_roll_velocity_));

    velocity_roll_ = velocity_roll_ltrigger + velocity_roll_rtrigger;
    
    velocity_pitch_ = std::abs(param_tilt_velocity_) < std::numeric_limits<double>::epsilon() ? 
                      0.0 :
                      joy2angle(tilt_direction_ * joy.axes[param_use_cross_ ? JoyKeymap::AXIS_CROSS_UD : JoyKeymap::AXIS_RSTICK_UD], 1.0, -1.0, std::abs(param_tilt_velocity_), - std::abs(param_tilt_velocity_));
    
    velocity_yaw_   = std::abs(param_pan_velocity_) < std::numeric_limits<double>::epsilon() ? 
                      0.0 :
                      joy2angle(pan_direction_ * joy.axes[param_use_cross_ ? JoyKeymap::AXIS_CROSS_LR : JoyKeymap::AXIS_RSTICK_LR], 1.0, -1.0, std::abs(param_pan_velocity_), - std::abs(param_pan_velocity_));
    
    reset_          = joy.buttons[JoyKeymap::BUTTON_RSTICK] == 1 ? true : false;

   // ROS_INFO("callback\n");
}


void PovCameraJoyController::updatePovCameraTransform()
{
    angle_roll_  = reset_ ? param_roll_init_ : angle2wrapped(angle_roll_  + velocity_roll_  / param_frequency_);
    angle_pitch_ = reset_ ? param_tilt_init_ : angle2wrapped(angle_pitch_ + velocity_pitch_ / param_frequency_);
    angle_yaw_   = reset_ ? param_pan_init_  : angle2wrapped(angle_yaw_   + velocity_yaw_   / param_frequency_);
    
    angle_roll_ = 0;
    //angle_pitch_ = 0; //angle_pitch_は上下
    //angle_yaw_ = 0; //angle_yaw_は左右
    tf::Quaternion tf_quaternion;
    tf_quaternion.setRPY(angle_roll_, angle_pitch_, angle_yaw_);
        
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf_quaternion, tf::Vector3(0.0, 0.0, 0.0)),
            ros::Time::now(),
            param_camera_frame_,
            param_pov_camera_frame_
        )
    );
     ROS_INFO("%f", angle_yaw_);
}


void PovCameraJoyController::loop()
{
    ros::Rate loop_rate(param_frequency_);
    
    while (nh_.ok()) {
        updatePovCameraTransform();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return;
}


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pov_camera_joy_controller");
    //ros::init(argc, argv, "pub_pan_angle");
    
    asterisk::PovCameraJoyController pov_camera_joy_controller;
    
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    
    pov_camera_joy_controller.loop();
    
    ros::waitForShutdown();
    
    return 0;
}
