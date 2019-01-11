#ifndef HOMERE_CONTROL__HOMERE_HARDWARE_INTERFACE_H
#define HOMERE_CONTROL__HOMERE_HARDWARE_INTERFACE_H

//
// This is ROS compatible hardware interface that uses beaglebone robotic hardware
// ( IMU, encoders) and a SaberTooth motor controller
//


#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "robotcontrol.h"

#include "homere_bbb/sabertooth.h"

#define NB_JOINTS 2

class HomereHardwareInterface : public hardware_interface::RobotHW
{
 public:
  HomereHardwareInterface();
  virtual ~HomereHardwareInterface();

  bool start(const ros::Time& time);
  void read(const ros::Time& time);
  void write();
  bool shutdown();
  //void switch_motors_on()  { rc_enable_motors(); }
  //void switch_motors_off() { rc_disable_motors(); }
  
  void IMUCallback(void);
 
 private:
  // Joints
  double joint_position_[NB_JOINTS];
  double joint_velocity_[NB_JOINTS];
  double joint_effort_[NB_JOINTS];
  double joint_effort_command_[NB_JOINTS];
  double joint_position_command_[NB_JOINTS];
  ros::Time last_enc_read_;
  
  // IMU
  hardware_interface::ImuSensorHandle::Data imu_data_;
  double imu_orientation_[4];         // quat is in the order of geometry_msg, ie x, y, z, w
  double imu_angular_velocity_[3];
  double imu_linear_acceleration_[3];

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::ImuSensorInterface     imu_sensor_interface_;

  rc_mpu_data_t rc_mpu_data_;
  // SaberTooth
  SaberTooth sabert_;
};


#endif // HOMERE_CONTROL__HOMERE_HARDWARE_INTERFACE_H
