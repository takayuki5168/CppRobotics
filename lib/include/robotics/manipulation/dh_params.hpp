#pragma once

namespace Robotics
{
  struct DHParams
  {
    DHParams(double a, double alpha, double d, double theta)
      : a(a), alpha(alpha), d(d), theta(theta)
    {}
    double a, alpha, d, theta;
  };
} // end namespace Roboics
