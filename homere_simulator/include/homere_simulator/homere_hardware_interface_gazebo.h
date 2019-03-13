#ifndef HOMERE_SIMULATOR__HOMERE_HARDWARE_INTERFACE_GAZEBO_H
#define HOMERE_SIMULATOR__HOMERE_HARDWARE_INTERFACE_GAZEBO_H

#include <ros/ros.h>
// ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <tf/LinearMath/Quaternion.h>

namespace homere_simulator {
#define NB_JOINTS 6
#define NB_CTL_JOINTS 6

  class HomereHardwareInterface : public gazebo_ros_control::RobotHWSim
  {
  public:
    HomereHardwareInterface();
    virtual bool initSim(
		 const std::string& robot_namespace,
		 ros::NodeHandle model_nh,
		 gazebo::physics::ModelPtr parent_model,
		 const urdf::Model *const urdf_model,
		 std::vector<transmission_interface::TransmissionInfo> transmissions);

    virtual void readSim(ros::Time time, ros::Duration period);

    virtual void writeSim(ros::Time time, ros::Duration period);

  private:
    gazebo::physics::ModelPtr model_;
    gazebo::physics::LinkPtr link_;
    //gazebo::math::Pose gz_pose_;
    //gazebo::math::Vector3 gz_velocity_, gz_acceleration_, gz_angular_velocity_;
    ignition::math::Pose3d gz_pose_;
    ignition::math::Vector3d gz_velocity_, gz_acceleration_, gz_angular_velocity_;

    gazebo::physics::JointPtr gz_joints_[NB_JOINTS];

    // Joints
    double joint_position_[NB_JOINTS];
    double joint_velocity_[NB_JOINTS];
    double joint_effort_[NB_JOINTS];
    double joint_effort_command_[NB_JOINTS];
    // IMU
    hardware_interface::ImuSensorHandle::Data imu_data_;
    double imu_orientation_[4]; // world_to_base
    double imu_angular_velocity_[3];
    double imu_linear_acceleration_[3];
    tf::Quaternion q_base_to_imu_;   // constant
    
    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::ImuSensorInterface     imu_sensor_interface_;
  };
}

  

#endif // HOMERE_SIMULATOR__HOMERE_HARDWARE_INTERFACE_GAZEBO_H
