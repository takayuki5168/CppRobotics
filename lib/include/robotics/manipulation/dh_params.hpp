#pragma once

namespace Robotics
{
  struct DHParams
  {
    DHParams(double theta, double alpha, double a, double d)
      : theta(theta), alpha(alpha), a(a), d(d) {}
    double theta, alpha, a, d;
  };
} // end namespace Roboics
