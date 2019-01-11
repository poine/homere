#include <homere_control/utils.h>

#include <iostream>
//#include <ros/ros.h>

#define __NAME "HomereControllerUtils"

namespace homere_controller {

 

  FeedForward::FeedForward():
    W1_(1,3), B1_(1,3), W2_(3,1), B2_(1,1) {
    //W1_ << -103.54677602,  -52.35031833,   -0.10817067;
    //B1_ << -0.10751237,  0.06115185,  6.0536281;
    //W2_ << 37.93297475, -75.03143049, -60.79612165;
    //B2_ << 371.99803428;
  }


  double FeedForward::get(double rvel) {
    Eigen::MatrixXd tmp1 = rvel * W1_ + B1_;
    for( int i = 0; i < tmp1.rows(); ++i )
      for (int j=0; j < tmp1.cols() ; ++j)
	if (tmp1(i,j) < 0.)
    	  tmp1(i,j) = 0.;
    Eigen::MatrixXd tmp2 = tmp1 * W2_ + B2_;
    //std::cerr << rvel << " " << tmp1 << " " << tmp1.rows() << tmp1.cols() << std::endl;
    //std::cerr << rvel << " " << tmp2 << std::endl;
    return tmp2(0,0);
  }


  // LeftFeedForward::LeftFeedForward() {
  //   W1_ << -14.16865123, -11.43596336,  -0.49962248;
  //   B1_ << 0.46397774, -0.45703255,  2.62417553;
  //   W2_ << -6.56283111, 8.12597734, -13.69903415;
  //   B2_ << 38.93937201;
  // }
  
  // RightFeedForward::RightFeedForward() {
  //   W1_ << -103.54677602,  -52.35031833,   -0.10817067;
  //   B1_ << -0.10751237,  0.06115185,  6.0536281;
  //   W2_ << 37.93297475, -75.03143049, -60.79612165;
  //   B2_ << 371.99803428;
  // }


}

#include "homere_control/generated/feedforward1.h"
