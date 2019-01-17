#ifndef HOMERE_CONTROL__ODOMETRY
#define HOMERE_CONTROL__ODOMETRY


#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace homere_controller
{
  namespace bacc = boost::accumulators;

  class Odometry
  {
  public:

    Odometry(size_t velocity_rolling_window_size = 10);
    void init(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
    void starting(const ros::Time &time);
    bool update(double left_pos, double right_pos, const ros::Time &time);

    void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
        return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
        return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
        return y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
        return linear_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
        return angular_;
    }

    double getLWrvel() const { return left_wheel_est_vel_; }
    double getRWrvel() const { return right_wheel_est_vel_; }
    void reset(double x, double y, double psi);

    /// Wheel kinematic parameters [m]:
    double wheel_separation_;
    double left_wheel_radius_;
    double right_wheel_radius_;
    
  private:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    void resetAccumulators();
 
    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_;  //   [m/s]
    double angular_; // [rad/s]

 

    /// Previou wheel position/state [rad]:
    double left_wheel_old_pos_;
    double right_wheel_old_pos_;

    /// Current wheel rotational velocities
    double left_wheel_est_vel_, right_wheel_est_vel_;

    
    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);
    
  };
}
  
#endif // HOMERE_CONTROL__ODOMETRY
