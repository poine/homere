#include <homere_control/homere_controller.h>

#include <math.h>
#include <Eigen/Dense>

#define __NAME "HomereController"
#define SAMPLE_RATE 100
// Filtering 
#define TIME_CONSTANT	0.5

// Control
#define WHEEL_KP 0.1
#define WHEEL_KI 0.3
#define WHEEL_KD 0.0
#define WHEEL_TF 0.007


namespace homere_controller {

  template<typename T>
  T clamp(T x, T min, T max)
  {
    return std::min(std::max(min, x), max);
  }
 
  HomereController::HomereController():
    left_wheel_duty_(0.),
    right_wheel_duty_(0.) {

    const double dt = 1.0/SAMPLE_RATE;
    rc_filter_butterworth_lowpass(&rvel_lp_l_, 2, dt, 2.0*M_PI/TIME_CONSTANT);
    rc_filter_butterworth_lowpass(&rvel_lp_r_, 2, dt, 2.0*M_PI/TIME_CONSTANT);
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

    double lw_kp, rw_kp, lw_ki, rw_ki, lw_kd, rw_kd;
    controller_nh.getParam("lw_kp", lw_kp);
    controller_nh.getParam("rw_kp", rw_kp);
    controller_nh.getParam("lw_ki", lw_ki);
    controller_nh.getParam("rw_ki", rw_ki);
    controller_nh.getParam("lw_kd", lw_kd);
    controller_nh.getParam("rw_kd", rw_kd);
    std::cerr << "Left Wheel PID kp:" << lw_kp << " ki:" << lw_ki << " kd:" << lw_kd << std::endl;
    std::cerr << "Righ Wheel PID kp:" << rw_kp << " ki:" << rw_ki << " kd:" << rw_kd << std::endl;
    const double dt = 1.0/SAMPLE_RATE;
    rc_filter_pid(&left_wheel_pid_, lw_kp, lw_ki, lw_kd, WHEEL_TF, dt);
    rc_filter_pid(&right_wheel_pid_, rw_kp, rw_ki, rw_kd, WHEEL_TF, dt);
    
    input_manager_.init(hw, controller_nh);

    double odom_ws, odom_lr, odom_rr;
    controller_nh.getParam("odom_ws", odom_ws);
    controller_nh.getParam("odom_lr", odom_lr);
    controller_nh.getParam("odom_rr", odom_rr);
    odometry_.init(odom_ws, odom_lr, odom_rr);
    std::cerr << "ODOM:" << odom_ws << " " << odom_lr << " " << odom_rr << std::endl;

    odom_publisher_.init(root_nh, controller_nh);
    //debug_publisher_.init(root_nh, controller_nh);
    debug_io_publisher_.init(root_nh, controller_nh);
    wr_debug_publisher_.init(root_nh, controller_nh);
    //raw_odom_publisher_.init(root_nh, controller_nh);
    reset_odom_srv_  = controller_nh.advertiseService("reset_odom",  &HomereController::OnOdomReset,  this);


    
    return true;
  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  //#define ODOM_PARAM_WHEEL_SEP 0.56
  //#define ODOM_PARAM_WHEEL_LR  0.19
  //#define ODOM_PARAM_WHEEL_RR 0.19
  void HomereController::starting(const ros::Time& now) {
    ROS_INFO_STREAM_NAMED(__NAME, "in HomereController::starting");
    //odometry_.setWheelParams(ODOM_PARAM_WHEEL_SEP, ODOM_PARAM_WHEEL_LR, ODOM_PARAM_WHEEL_RR);
    odometry_.starting(now);
    odom_publisher_.starting(now);
    wr_debug_publisher_.starting(now);
    //hw_->switch_motors_on();
  }

   /*******************************************************************************
   *
   *
   *******************************************************************************/
  void HomereController::update(const ros::Time& now, const ros::Duration& dt) {
    double lw_angle = left_wheel_joint_.getPosition();
    double rw_angle = right_wheel_joint_.getPosition();
    odometry_.update(lw_angle, rw_angle, now);

    double lw_rvel = left_wheel_joint_.getVelocity();
    double lw_rvel_f = rc_filter_march(&rvel_lp_l_, lw_rvel);

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
      lw_rvel_sp_ = input_manager_.rt_commands_.rvel_l;
      rw_rvel_sp_ = input_manager_.rt_commands_.rvel_r;
      compute_wheel_control(lw_rvel_sp_, rw_rvel_sp_, lw_angle, rw_angle, lw_rvel, rw_rvel, dt);
   
    }
    else if (mode == 42) { // lin and ang vel
      double lin_sp = input_manager_.rt_commands_.lin;
      double ang_sp = input_manager_.rt_commands_.ang;
      // Compute wheels velocities:
      lw_rvel_sp_ = (lin_sp - ang_sp * odometry_.wheel_separation_ / 2.0) / odometry_.left_wheel_radius_;
      rw_rvel_sp_ = (lin_sp + ang_sp * odometry_.wheel_separation_ / 2.0) / odometry_.right_wheel_radius_;
      compute_wheel_control(lw_rvel_sp_, rw_rvel_sp_, lw_angle, rw_angle, lw_rvel, rw_rvel, dt);
    }
    //std::printf("Sending %f %f\n", left_wheel_duty_, right_wheel_duty_);
    left_wheel_joint_.setCommand(left_wheel_duty_);
    right_wheel_joint_.setCommand(right_wheel_duty_);
    //    debug_publisher_.publish(lw_rvel_sp_, rw_rvel_sp_, lw_angle, rw_angle, lw_rvel, rw_rvel,
    //			     lw_rvel_f, rw_rvel_f, left_wheel_duty_, right_wheel_duty_, now);
    //debug_publisher_.publish(lw_ref_.rvel_, rw_ref_.rvel_, lw_angle, rw_angle, lw_rvel, rw_rvel,
    //			     lw_rvel_f, rw_rvel_f, left_wheel_duty_, right_wheel_duty_, now);
    debug_io_publisher_.publish(lw_angle, rw_angle, lw_rvel, rw_rvel, left_wheel_duty_, right_wheel_duty_, now);
    wr_debug_publisher_.publish(lw_rvel_sp_, rw_rvel_sp_,
				lw_ref_.angle_, rw_ref_.angle_,
				lw_ref_.rvel_, rw_ref_.rvel_,
				lw_ref_.rveld_, rw_ref_.rveld_,
				now);
    odom_publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(),
		       odometry_.getLinear(), odometry_.getAngular(), now);
  }



  void HomereController::compute_wheel_control(double lw_rvel_sp, double rw_rvel_sp,
					       double lw_angle, double rw_angle,
					       double lw_rvel, double rw_rvel,
					       const ros::Duration& dt) {
    lw_ref_.update(lw_rvel_sp, dt.toSec());
    rw_ref_.update(rw_rvel_sp, dt.toSec());

    double lw_angl_err = lw_ref_.angle_ - lw_angle;
    double rw_angl_err = rw_ref_.angle_ - rw_angle; 
    
    double lw_rvel_err = lw_ref_.rvel_ - lw_rvel; //lw_rvel_sp - lw_rvel;
    double rw_rvel_err = rw_ref_.rvel_ - rw_rvel; //rw_rvel_sp - rw_rvel;

    // fine

#ifdef RCPID
    double lw_feedback = rc_filter_march(&left_wheel_pid_, lw_rvel_err);
    double rw_feedback = rc_filter_march(&right_wheel_pid_, rw_rvel_err);
#else    
    //
    lw_ang_sum_err_ += lw_angl_err;
    rw_ang_sum_err_ += rw_angl_err;
    const double Ki = 0.0, Kp = 1.5, Kd = 20;
    double lw_feedback = Ki * lw_ang_sum_err_ + Kp * lw_angl_err + Kd * lw_rvel_err;
    double rw_feedback = Ki * rw_ang_sum_err_ + Kp * rw_angl_err + Kd * rw_rvel_err;
#endif
    double lw_feedforward = lw_ref_.rveld_ * 20.;//0;//lw_feedforward_.get(lw_rvel_sp);
    double rw_feedforward = rw_ref_.rveld_ * 20.;//0;//rw_feedforward_.get(rw_rvel_sp);

    left_wheel_duty_  =  lw_feedforward + lw_feedback;
    right_wheel_duty_ =  rw_feedforward + rw_feedback;

    left_wheel_duty_ = clamp(left_wheel_duty_, -50., 50.);
    right_wheel_duty_ = clamp(right_wheel_duty_, -50., 50.);
    
  }

  // rosservice call /homere/homere_controller/reset_odom 0. 0. 0.
  bool HomereController::OnOdomReset(homere_control::srv_reset_odom::Request  &req, homere_control::srv_reset_odom::Response &res) {
    ROS_INFO_STREAM_NAMED(__NAME, "in HomereController::OnOdomReset");
    odometry_.reset(req.x0, req.y0, req.psi0, left_wheel_joint_.getPosition(), right_wheel_joint_.getPosition());
    return true;
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
