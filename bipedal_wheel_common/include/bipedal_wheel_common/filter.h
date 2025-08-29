//
// Created by guanlin on 25-8-29.
//

#pragma once

#include <cmath>

class RampFilter
{
public:
  RampFilter(double acc, double dt) : acc_(acc), dt_(dt), last_value_(0.) {}
  ~RampFilter() = default;
  void setAcc(double acc) { acc_ = acc; }
  void input(double input_value) { last_value_ += minAbs(input_value - last_value_, acc_ * dt_); }
  double output() { return last_value_; }

private:
  double minAbs(double a, double b)
  {
    double sign = (a < 0.0) ? -1.0 : 1.0;
    return sign * fmin(fabs(a), b);
  }
  double last_value_;
  double acc_;
  double dt_;
};
