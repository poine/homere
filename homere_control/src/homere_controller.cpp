#include <homere_control/homere_controller.h>

#include <math.h>

#define __NAME "HomereController"
#define SAMPLE_RATE 100
#define TIME_CONSTANT	1.0

// Control
#define WHEEL_KP 0
#define WHEEL_KI 0.01
#define WHEEL_KD 0
#define WHEEL_TF 0.007
//#define WHEEL_DT (1./SAMPLE_RATE)

namespace homere_controller {

  HomereController::HomereController():
    left_wheel_duty_(0.),
    right_wheel_duty_(0.) {

    const double dt = 1.0/SAMPLE_RATE;
    rc_filter_butterworth_lowpass(&rvel_lp_l_, 2, dt, 2.0*M_PI/TIME_CONSTANT);
    rc_filter_butterworth_lowpass(&rvel_lp_r_, 2, dt, 2.0*M_PI/TIME_CONSTANT);
    rc_filter_pid(&left_wheel_pid_, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_TF, dt);
    rc_filter_pid(&right_wheel_pid_, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_TF, dt);
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

    input_manager_.init(hw, controller_nh);
    //odometry_.init(WHEEL_BASE, VELOCITY_ROLLING_WINDOW_SIZE);
    publisher_.init(root_nh, controller_nh);
    debug_publisher_.init(root_nh, controller_nh);
    //raw_odom_publisher_.init(root_nh, controller_nh);
    return true;
  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::starting(const ros::Time& now) {
    ROS_INFO_STREAM_NAMED(__NAME, "in HomereController::starting");
    odometry_.setWheelParams(0.56, 0.19, 0.19);
    odometry_.init(now);
    //odometry_.starting(now);
    publisher_.starting(now);
    //hw_->switch_motors_on();
  }

   /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::update(const ros::Time& now, const ros::Duration& dt) {
    double left_pos = left_wheel_joint_.getPosition();
    double right_pos = right_wheel_joint_.getPosition();
    odometry_.update(left_pos, right_pos, now);

    //double lw_rvel_f = rc_filter_march(&rvel_lp_l_, odometry_.getLWrvel());
    double lw_rvel = left_wheel_joint_.getVelocity();
    double lw_rvel_f = rc_filter_march(&rvel_lp_l_, lw_rvel);
    //double rw_rvel_f = rc_filter_march(&rvel_lp_r_, odometry_.getRWrvel());
    double rw_rvel = right_wheel_joint_.getVelocity();
    double rw_rvel_f = rc_filter_march(&rvel_lp_r_, rw_rvel);
      
    input_manager_.update(now);
    //const double t0 = now.toSec();
    //double s = 10.*sin(0.5*t0);
    int mode = input_manager_.rt_commands_.mode;
    if (mode == 0) {  // direct PWM (between -127 and 127)
      left_wheel_duty_ =  input_manager_.rt_commands_.pwm_l;
      right_wheel_duty_ = input_manager_.rt_commands_.pwm_r;
    }
    else if (mode == 1) { // wheel rvel (in rad/s)
      double lw_rvel_sp = input_manager_.rt_commands_.rvel_l;
      double rw_rvel_sp = input_manager_.rt_commands_.rvel_r;

      double lw_err = lw_rvel_sp - lw_rvel;
      double rw_err = rw_rvel_sp - rw_rvel;
    
      double lw_feedback = rc_filter_march(&left_wheel_pid_, lw_err);
      double rw_feedback = rc_filter_march(&right_wheel_pid_, rw_err);

      left_wheel_duty_  =  lw_rvel_sp*10;// lw_feedback + 
      right_wheel_duty_ =  rw_rvel_sp*10;// rw_feedback + 
    }
    //std::printf("Sending %f %f\n", left_wheel_duty_, right_wheel_duty_);
    left_wheel_joint_.setCommand(left_wheel_duty_);
    right_wheel_joint_.setCommand(right_wheel_duty_);
    debug_publisher_.publish(left_pos, right_pos, lw_rvel, rw_rvel, lw_rvel_f, rw_rvel_f, left_wheel_duty_, right_wheel_duty_, now);
    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::stopping(const ros::Time&) {
    left_wheel_joint_.setCommand(0);
    right_wheel_joint_.setCommand(0);
  }

  PLUGINLIB_EXPORT_CLASS(homere_controller::HomereController, controller_interface::ControllerBase);
}
