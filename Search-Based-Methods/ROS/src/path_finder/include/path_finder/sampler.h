/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _BIAS_SAMPLER_
#define _BIAS_SAMPLER_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>

class BiasSampler
{
public:
  BiasSampler()
  {
    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    // gen_ = std::mt19937_64(1234);
    uniform_rand_ = std::uniform_real_distribution<double>(0.0, 1.0);
    normal_rand_ = std::normal_distribution<double>(0.0, 1.0);
    range_.setZero();
    origin_.setZero();
  };

  void setSamplingRange(const Eigen::Vector3d origin, const Eigen::Vector3d range)
  {
    origin_ = origin;
    range_ = range;
  }

  void samplingOnce(Eigen::Vector3d &sample)
  {
    sample[0] = uniform_rand_(gen_);
    sample[1] = uniform_rand_(gen_);
    sample[2] = uniform_rand_(gen_);
    sample.array() *= range_.array();
    sample += origin_;
  };

  void informed_samplingOnce(Eigen::Vector3d &sample,Eigen::Vector3d start_,Eigen::Vector3d goal_, double c_best_)
  {
    if(c_best_){
      Eigen::Vector3d v0(1,0,0);
      Eigen::Vector3d v1=goal_-start_;
      // double angle= acos(ax_ori.dot(ax_new)/ax_new.norm()/ax_ori.norm());
      Eigen::Matrix3d C(Eigen::Quaterniond::FromTwoVectors(v0,v1));


      double c_best_sqr=c_best_*c_best_;
      double c_min_sqr = (start_-goal_).transpose().dot(start_-goal_);
      double PI    = 3.1415926535;
      double r     = uniform_rand_(gen_);
      double theta = PI*uniform_rand_(gen_);
      double phi   = 2*PI*uniform_rand_(gen_);
      Eigen::Vector3d x_ball(r*sin(theta)*cos(phi), r*sin(theta)*sin(phi), r*cos(theta));
      Eigen::Vector3d x_centre = 0.5*(start_ + goal_);
      double temp(sqrt(c_best_sqr-c_min_sqr));
      Eigen::DiagonalMatrix<double, 3> L(0.5*c_best_, 0.5*temp, 0.5*temp);
      sample = C*L*x_ball+x_centre;
    }
    else{
      sample[0] = uniform_rand_(gen_);
      sample[1] = uniform_rand_(gen_);
      sample[2] = uniform_rand_(gen_);
      sample.array() *= range_.array();
      sample += origin_;
    }
  };

  // (0.0 - 1.0)
  double getUniRandNum()
  {
    return uniform_rand_(gen_);
  }

private:
  Eigen::Vector3d range_, origin_;
  std::mt19937_64 gen_;
  std::uniform_real_distribution<double> uniform_rand_;
  std::normal_distribution<double> normal_rand_;
};

#endif
