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
  
}

#endif // HOMERE_CONTROL__UTILS_H
