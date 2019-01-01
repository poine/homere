#include <homere_control/homere_controller.h>
#define __NAME "HomereController"
namespace homere_controller {

  HomereController::HomereController():
    left_wheel_duty_(0.),
    right_wheel_duty_(0.) {
    
  }

  HomereController::~HomereController()      {

  }


  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool HomereController::init(hardware_interface::RobotHW* hw,
			      ros::NodeHandle& root_nh,
			      ros::NodeHandle& controller_nh)
  {
    ROS_INFO_STREAM_NAMED(__NAME, "in HomereController::init");
    //hw_ = static_cast<OscarHardwareInterface*>(hw);
    hardware_interface::EffortJointInterface* e = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e->getHandle("left_wheel_joint");
    right_wheel_joint_ = e->getHandle("right_wheel_joint");

    //input_manager_.init(hw, controller_nh);
    //odometry_.init(WHEEL_BASE, VELOCITY_ROLLING_WINDOW_SIZE);
    //publisher_.init(root_nh, controller_nh);
    //raw_odom_publisher_.init(root_nh, controller_nh);
    return true;
  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::starting(const ros::Time& now) {
    ROS_INFO_STREAM_NAMED(__NAME, "in HomereController::starting");
    //odometry_.starting(now);
    //publisher_.starting(now);
    //hw_->switch_motors_on();
  }

   /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::update(const ros::Time& now, const ros::Duration& dt) {
    double left_wheel_duty_ = 10.;
    double right_wheel_duty_ = 10.;
    left_wheel_joint_.setCommand(left_wheel_duty_);
    right_wheel_joint_.setCommand(right_wheel_duty_);
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::stopping(const ros::Time&) {
    
  }

  PLUGINLIB_EXPORT_CLASS(homere_controller::HomereController, controller_interface::ControllerBase);
}
