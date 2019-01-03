#pragma once


/*
 * TODO add max min of dh parameters
 */

#include "robotics/manipulation/dh_params.hpp"

namespace Robotics
{
  class AbstJoint
  {
  public:
    AbstJoint(double dh_a, double dh_alpha, double dh_d, double dh_theta)
      : dh_params_(DHParams(dh_a, dh_alpha, dh_d, dh_theta)) {}

    AbstJoint(DHParams dh_params)
      : dh_params_(dh_params) {}

    /*
     * use current DH parameters
     */
    Eigen::Matrix4d calcTransformationMatrix() const
    {
      const double dh_a = dh_params_.a;
      const double dh_alpha = dh_params_.alpha;
      const double dh_d = dh_params_.d;
      const double dh_theta = dh_params_.theta;
      
      Eigen::Matrix4d trans;
      trans << std::cos(dh_theta), -std::sin(dh_theta), 0, dh_a,
	std::cos(dh_alpha) * std::sin(dh_theta), std::cos(dh_alpha) * std::cos(dh_theta), -std::sin(dh_alpha), -dh_d * std::sin(dh_alpha),
	std::sin(dh_alpha) * std::sin(dh_theta), std::sin(dh_alpha) * std::cos(dh_theta), std::cos(dh_alpha), dh_d * std::cos(dh_alpha),      
	0, 0, 0, 1;

      return trans;
    }

    /*
     * use designated DH parameters, not current ones
     */
    Eigen::Matrix4d calcTransformationMatrix(double dh_a, double dh_alpha, double dh_d, double dh_theta) const
    {
      Eigen::Matrix4d trans;
      trans << std::cos(dh_theta), -std::sin(dh_theta), 0, dh_a,
	std::cos(dh_alpha) * std::sin(dh_theta), std::cos(dh_alpha) * std::cos(dh_theta), -std::sin(dh_alpha), -dh_d * std::sin(dh_alpha),
	std::sin(dh_alpha) * std::sin(dh_theta), std::sin(dh_alpha) * std::cos(dh_theta), std::cos(dh_alpha), dh_d * std::cos(dh_alpha),      
	0, 0, 0, 1;

      return trans;
    }

    DHParams getDHParams() const { return dh_params_; }

  protected:
    DHParams dh_params_;
  };
} // end namespace Robotics
