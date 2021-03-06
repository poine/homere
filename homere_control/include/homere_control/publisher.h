#ifndef HOMERE_CONTROL__PUBLISHER_H_
#define HOMERE_CONTROL__PUBLISHER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <homere_control/homere_controller_debug.h>
#include <homere_control/debug_io.h>
#include <homere_control/debug_wheel_ref.h>

#define MAX_SENSOR_LEN 15
#define MIN_SENSOR_FOR_PUBLISH 10

namespace homere_controller {

  //
  // Publish odometry
  //
  class OdomPublisher {

  public:
    OdomPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& now);
    void publish(const double heading, const double x, const double y, const double linear, const double angular, const ros::Time& now);

  private:
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string base_link_;
    bool enable_odom_tf_;
 
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

  };


  //
  // Publish input/state/output 
  //
  class DebugPublisher {

  public:
    DebugPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void publish(const double lw_rvel_sp, const double rw_rvel_sp,
		 const double lw_angle, const double rw_angle,
		 const double lw_rvel, const double rw_rvel,
		 const double lw_rvel_f, const double rw_rvel_f,
		 const int8_t lw_pwm, const int8_t rw_pwm, const ros::Time& now);

  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<homere_control::homere_controller_debug> > pub_;
    int nb_data_;
    ros::Time stamp_[MAX_SENSOR_LEN];
    double lw_rvel_sp_[MAX_SENSOR_LEN];
    double rw_rvel_sp_[MAX_SENSOR_LEN];
    double lw_angle_[MAX_SENSOR_LEN];
    double rw_angle_[MAX_SENSOR_LEN];
    double lw_rvel_[MAX_SENSOR_LEN];
    double rw_rvel_[MAX_SENSOR_LEN];
    double lw_rvel_f_[MAX_SENSOR_LEN];
    double rw_rvel_f_[MAX_SENSOR_LEN];
    int8_t lw_pwm_[MAX_SENSOR_LEN];
    int8_t rw_pwm_[MAX_SENSOR_LEN];
  };


  //
  // Publish input/output 
  //
  class DebugIOPublisher {

  public:
    DebugIOPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void publish(const double lw_angle, const double rw_angle,
		 const double lw_rvel, const double rw_rvel,
		 const int8_t lw_pwm, const int8_t rw_pwm, const ros::Time& now);

  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<homere_control::debug_io> > pub_;
    int nb_data_;
    ros::Time stamp_[MAX_SENSOR_LEN];
    double lw_angle_[MAX_SENSOR_LEN];
    double rw_angle_[MAX_SENSOR_LEN];
    double lw_rvel_[MAX_SENSOR_LEN];
    double rw_rvel_[MAX_SENSOR_LEN];
    int8_t lw_pwm_[MAX_SENSOR_LEN];
    int8_t rw_pwm_[MAX_SENSOR_LEN];
  };



  

  //
  // Publish wheel controller reference
  //
  class WRDebugPublisher {
  public:
    WRDebugPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& now);
    void publish(const double lw_rvel_sp,  const double rw_rvel_sp,
		 const double lw_angl_ref, const double rw_angl_ref,
		 const double lw_rvel_ref, const double rw_rvel_ref,
		 const double lw_rveld_ref, const double rw_rveld_ref,
		 const ros::Time& now);
  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<homere_control::debug_wheel_ref> > pub_;
    ros::Duration publish_period_;
    ros::Time last_publish_time_;
  };
  
}

#endif // HOMERE_CONTROL__PUBLISHER_H_
