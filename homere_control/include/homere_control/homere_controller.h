#ifndef HOMERE_CONTROL__HOMERE_CONTROLLER_H
#define HOMERE_CONTROL__HOMERE_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <pluginlib/class_list_macros.h>

#include "robotcontrol.h"

#include "homere_control/input_manager.h"
#include "homere_control/odometry.h"
#include "homere_control/publisher.h"

namespace homere_controller {

  class HomereController :
    public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::ImuSensorInterface>
    {
    public:
      HomereController();
      ~HomereController();
      
      bool init(hardware_interface::RobotHW* hw,
		ros::NodeHandle& root_nh,
		ros::NodeHandle &controller_nh);
      void starting(const ros::Time& time);
      void update(const ros::Time& , const ros::Duration&);
      void stopping(const ros::Time&);

    private:
      hardware_interface::JointHandle left_wheel_joint_;
      hardware_interface::JointHandle right_wheel_joint_;

      homere_controller::InputManager   input_manager_;
      homere_controller::Odometry       odometry_;
      homere_controller::Publisher      publisher_;
      homere_controller::DebugPublisher debug_publisher_;
      
      // filter structures
      rc_filter_t rvel_lp_l_, rvel_lp_r_;
      rc_filter_t left_wheel_pid_, right_wheel_pid_;
      
      // values output to the hardware interface
      double left_wheel_duty_, right_wheel_duty_;
      
    };
}

#endif // HOMERE_CONTROL__HOMERE_CONTROLLER_H
