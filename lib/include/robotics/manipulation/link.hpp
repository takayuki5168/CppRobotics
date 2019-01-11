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
      : dh_params_(DHParams(dh_theta, dh_alpha, dh_a, dh_d)), joint_type_(joint_type), dh_params_changed_(true) {}

    Link(DHParams dh_params, JointType joint_type)
      : dh_params_(dh_params), joint_type_(joint_type), dh_params_changed_(true) {}

    Eigen::Matrix4d calcTransformationMatrix()
    {
      if (dh_params_changed_) {
	const double dh_theta = dh_params_.theta;
	const double dh_alpha = dh_params_.alpha;      
	const double dh_a = dh_params_.a;
	const double dh_d = dh_params_.d;
      
	trans_ << std::cos(dh_theta), -std::sin(dh_theta), 0, dh_a,
	  std::cos(dh_alpha) * std::sin(dh_theta), std::cos(dh_alpha) * std::cos(dh_theta), -std::sin(dh_alpha), -dh_d * std::sin(dh_alpha),
	  std::sin(dh_alpha) * std::sin(dh_theta), std::sin(dh_alpha) * std::cos(dh_theta), std::cos(dh_alpha), dh_d * std::cos(dh_alpha),      
	  0, 0, 0, 1;

	dh_params_changed_ = false;	
      }
      
      return trans_;
    }

    Eigen::VectorXd calcBasicJacobian(double joint_angle, Eigen::Matrix4d trans, Eigen::Vector3d end_effector_pos) const
    {
      Eigen::Vector3d pos = trans.block(0, 3, 3, 1);
      Eigen::Vector3d z_axis = trans.block(0, 2, 3, 1);

      Eigen::VectorXd basic_jacobian = Eigen::VectorXd::Zero(6);
      basic_jacobian.block(0, 0, 3, 1) = z_axis.cross(end_effector_pos - pos);
      basic_jacobian.block(3, 0, 3, 1) = z_axis;
      return basic_jacobian;
    }

    DHParams getDHParams() const { return dh_params_; }
    void setJointAngle(double joint_angle) {
      dh_params_.theta = joint_angle;
      dh_params_changed_ = true;
    }
    
    JointType getJointType() const { return joint_type_; }

  protected:
    DHParams dh_params_;
    JointType joint_type_;
    Eigen::Matrix4d trans_;
    bool dh_params_changed_;
  };
} // end namespace Robotics
