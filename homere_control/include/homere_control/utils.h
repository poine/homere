#ifndef HOMERE_CONTROL__UTILS_H
#define HOMERE_CONTROL__UTILS_H

#include <Eigen/Dense>

namespace homere_controller {

  // This is a 2 layers feedforward perceptron
  // Weights and Biases are computed using sklearn
  class FeedForward {
  public:
    FeedForward();
    double get(double rvel);
  protected:
    Eigen::MatrixXd W1_;
    Eigen::MatrixXd B1_;
    Eigen::MatrixXd W2_;
    Eigen::MatrixXd B2_;
  };

  class LeftFeedForward: public FeedForward {
  public:
    LeftFeedForward();
  };

  class RightFeedForward: public FeedForward {
  public:
    RightFeedForward();
  };

  // Reference model for wheel
  class WheelRef {
  public:
    WheelRef(double omega=1.5, double xi=0.9);
    void update(const double& setpoint, const double& dt);
    double angle_;
    double rvel_;
    double rveld_;
    double rveldd_;
  private:
    double omega_;
    double xi_;
  };

  
}

#endif // HOMERE_CONTROL__UTILS_H
