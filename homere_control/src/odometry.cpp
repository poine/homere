#include <homere_control/odometry.h>
//
// original:
// https://github.com/ros-controls/ros_controllers/blob/melodic-devel/diff_drive_controller/src/odometry.cpp
//

namespace homere_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , wheel_separation_(0.0)
  , left_wheel_radius_(0.0)
  , right_wheel_radius_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , left_wheel_est_vel_(0.), right_wheel_est_vel_(0.)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size) {

  }

  void Odometry::init(double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  }
  
  void Odometry::starting(const ros::Time& time) {
      // Reset accumulators and timestamp:
      resetAccumulators();
      timestamp_ = time;
  }

  void Odometry::starting(const double time) {
      // Reset accumulators and timestamp:
      //resetAccumulators();
      //timestamp_ = ros::Time(time);
      starting(ros::Time(time));
  }

  bool Odometry::update(double left_pos, double right_pos, const double  time) {
    return update(left_pos, right_pos, ros::Time(time));
  }
  
  bool Odometry::update(double left_pos, double right_pos, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * left_wheel_radius_;
    const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    //const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    //const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;
    left_wheel_est_vel_  = left_wheel_cur_pos  - left_wheel_old_pos_;
    right_wheel_est_vel_ = right_wheel_cur_pos - right_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel_ + left_wheel_est_vel_) * 0.5 ;
    const double angular = (right_wheel_est_vel_ - left_wheel_est_vel_) / wheel_separation_;

    integrateExact(linear, angular);
  
    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }
  

  void Odometry::reset(double x, double y, double psi, double left_pos, double right_pos) {
    x_ = x; y_=y; heading_=psi;
    left_wheel_old_pos_  = left_pos  * left_wheel_radius_;
    right_wheel_old_pos_ = right_pos * right_wheel_radius_;
    std::cerr << " reset " << x_ << " " << y_ << std::endl;
  }
  
  void Odometry::setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    wheel_separation_   = wheel_separation;
    left_wheel_radius_  = left_wheel_radius;
    right_wheel_radius_ = right_wheel_radius;
  }
  
  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }


  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }


}
