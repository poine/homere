#include "homere_control/input_manager.h"


namespace homere_controller {

  InputManager::InputManager() {
    
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool InputManager::init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh) {
    sub_command_vel_ = controller_nh.subscribe("cmd_vel", 1, &InputManager::cmdVelCallback, this);
    sub_command_ = controller_nh.subscribe("cmd", 1, &InputManager::cmdCallback, this);
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/

  void InputManager::update(const ros::Time& now) {
    
    rt_commands_ = *(command_.readFromRT());
    const double dt = (now - rt_commands_.stamp).toSec();
    if (dt > 0.5) {
      rt_commands_.mode = 0;
      rt_commands_.lin = 0.;
      rt_commands_.ang = 0.;
      rt_commands_.pwm_l = 0;
      rt_commands_.pwm_r = 0;
    }
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void InputManager::cmdVelCallback(const geometry_msgs::Twist& command) {
    nrt_ros_command_struct_.ang   = command.angular.z;
    nrt_ros_command_struct_.lin   = command.linear.x;
    nrt_ros_command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (nrt_ros_command_struct_);
  }

  void InputManager::cmdCallback(const homere_control::homere_controller_input& command) {
    nrt_ros_command_struct_.mode   = command.mode;
    nrt_ros_command_struct_.pwm_l   = command.pwm_l;
    nrt_ros_command_struct_.pwm_r   = command.pwm_r;
    nrt_ros_command_struct_.rvel_l   = command.rvel_l;
    nrt_ros_command_struct_.rvel_r   = command.rvel_r;
    nrt_ros_command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (nrt_ros_command_struct_);
  }
}
