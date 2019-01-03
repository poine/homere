#ifndef HOMERE_CONTROL__INPUT_MANAGER
#define HOMERE_CONTROL__INPUT_MANAGER

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/robot_hw.h>

#include <homere_control/homere_controller_input.h>

namespace homere_controller {

  class InputManager {

  public:
    InputManager();
    bool init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh);
    void update(const ros::Time&);
 
    struct Commands
    {
      uint8_t mode;
      int pwm_l;
      int pwm_r;
      double rvel_l;
      double rvel_r;
      double lin;
      double ang;
      ros::Time stamp;
      
    Commands() : mode(0), pwm_l(0), pwm_r(0), rvel_l(0.), rvel_r(0.), lin(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands nrt_ros_command_struct_;
    ros::Subscriber sub_command_vel_;
    ros::Subscriber sub_command_;

    Commands rt_commands_;
    
  private:
    void cmdVelCallback(const geometry_msgs::Twist& command);
    void cmdCallback(const homere_control::homere_controller_input& command);
  };

}




#endif // HOMERE_CONTROL__INPUT_MANAGER
