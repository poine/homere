#include <homere_simulator/homere_hardware_interface_gazebo.h>


namespace homere_simulator {

  const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint",
					      "left_caster_ax1_to_ax2_joint", "right_caster_ax1_to_ax2_joint",
					      "left_caster_fork_to_wheel_joint", "right_caster_fork_to_wheel_joint"};

  HomereHardwareInterface::HomereHardwareInterface() {
    this->registerInterface(static_cast<HomereHardwareInterface *>(this));
  }

  
  bool HomereHardwareInterface::initSim(
					const std::string& robot_namespace,
					ros::NodeHandle model_nh,
					gazebo::physics::ModelPtr parent_model,
					const urdf::Model *const urdf_model,
					std::vector<transmission_interface::TransmissionInfo> transmissions) {
    // store parent model pointer
    model_ = parent_model;
    link_ = model_->GetLink();
    for (int i=0; i<NB_JOINTS; i++) {
      gz_joints_[i] = parent_model->GetJoint(joint_name_[i]);
    }
   // register joints state
    for (int i=0; i<NB_JOINTS; i++) {
      joint_position_[i] = joint_velocity_[i] = joint_effort_[i] = 0.;
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
	joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    }
    registerInterface(&js_interface_);
    // register joints effort
    for (int i=0; i<NB_CTL_JOINTS; i++) {
      joint_effort_command_[i] = 0.;
      ej_interface_.registerHandle(hardware_interface::JointHandle(
        js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
    }
    registerInterface(&ej_interface_);
    // register IMU
    imu_data_.name = "imu";
    imu_data_.frame_id = "imu_link";
    imu_data_.orientation = imu_orientation_;
    imu_data_.angular_velocity = imu_angular_velocity_;
    imu_data_.linear_acceleration = imu_linear_acceleration_;
    hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    registerInterface(&imu_sensor_interface_);
    q_base_to_imu_.setRPY(0, 0, 0);
 
    return true;   
  }

  
  void HomereHardwareInterface::readSim(ros::Time time, ros::Duration period) {
    for (int i=0; i<NB_JOINTS; i++) {
      //joint_position_[i] = gz_joints_[i]->GetAngle(0).Radian();
      joint_position_[i] = gz_joints_[i]->Position(0);
      joint_velocity_[i] = gz_joints_[i]->GetVelocity(0);
    }
    //gz_pose_ =  link_->GetWorldPose();
    gz_pose_ =  link_->WorldPose();
    //tf::Quaternion q_world_to_base = tf::Quaternion(gz_pose_.rot.x, gz_pose_.rot.y, gz_pose_.rot.z, gz_pose_.rot.w); // x, y, z, w
    tf::Quaternion q_world_to_base = tf::Quaternion(gz_pose_.Rot().X(), gz_pose_.Rot().Y(), gz_pose_.Rot().Z(), gz_pose_.Rot().W()); // x, y, z, w
    tf::Quaternion q_world_to_imu = q_world_to_base * q_base_to_imu_;
    imu_orientation_[0] = q_world_to_imu.x();
    imu_orientation_[1] = q_world_to_imu.y();
    imu_orientation_[2] = q_world_to_imu.z();
    imu_orientation_[3] = q_world_to_imu.w();
    
  }

#define pwm_to_force 0.01
  void HomereHardwareInterface::writeSim(ros::Time time, ros::Duration period) {
    for (int i=0; i<NB_CTL_JOINTS; i++) {
      gz_joints_[i]->SetForce(0, joint_effort_command_[i]*pwm_to_force);
    }
  }

  
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(homere_simulator::HomereHardwareInterface, gazebo_ros_control::RobotHWSim)
