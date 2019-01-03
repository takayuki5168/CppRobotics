#pragma once

/*
 * TODO add max min of dh parameters
 */

#include <Eigen/Geometry>
#include "robotics/manipulation/dh_params.hpp"
#include "robotics/manipulation/joint_type.hpp"

namespace Robotics
{
  class Link
  {
  public:
    Link(double dh_theta, double dh_alpha, double dh_a, double dh_d, JointType joint_type)
      : dh_params_(DHParams(dh_theta, dh_alpha, dh_a, dh_d)), joint_type_(joint_type) {}

    Link(DHParams dh_params, JointType joint_type)
      : dh_params_(dh_params), joint_type_(joint_type) {}

    /*
     * use current DH parameters
     */
    Eigen::Matrix4d calcTransformationMatrix() const
    {
      const double dh_theta = dh_params_.theta;
      const double dh_alpha = dh_params_.alpha;      
      const double dh_a = dh_params_.a;
      const double dh_d = dh_params_.d;
      
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
    Eigen::Matrix4d calcTransformationMatrix(double dh_theta, double dh_alpha, double dh_a, double dh_d) const
    {
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
    Eigen::Matrix4d calcTransformationMatrix(double dh_val) const
    {
      const double dh_theta = dh_params_.theta;
      const double dh_alpha = dh_params_.alpha;      
      const double dh_a = dh_params_.a;
      const double dh_d = dh_params_.d;
      
      Eigen::Matrix4d trans;
      switch (joint_type_){
      case JointType::Rotational:
	trans << std::cos(dh_val), -std::sin(dh_val), 0, dh_a,
	  std::cos(dh_alpha) * std::sin(dh_val), std::cos(dh_alpha) * std::cos(dh_val), -std::sin(dh_alpha), -dh_d * std::sin(dh_alpha),
	  std::sin(dh_alpha) * std::sin(dh_val), std::sin(dh_alpha) * std::cos(dh_val), std::cos(dh_alpha), dh_d * std::cos(dh_alpha),      
	  0, 0, 0, 1;
	break;
      case JointType::Linear:
	break;
      default:
	break;
      }

      return trans;
    }

    DHParams getDHParams() const { return dh_params_; }
    JointType getJointType() const { return joint_type_; }

  protected:
    DHParams dh_params_;
    JointType joint_type_;
    //Link parent_link;
  };
} // end namespace Robotics
