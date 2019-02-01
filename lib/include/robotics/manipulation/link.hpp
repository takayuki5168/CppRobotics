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

    Eigen::Matrix4d transformationMatrix()
    {
      if (dh_params_changed_) {
	const double st = std::sin(dh_params_.theta);
	const double ct = std::cos(dh_params_.theta);
	const double sa = std::sin(dh_params_.alpha);
	const double ca = std::cos(dh_params_.alpha);	
	const double a = dh_params_.a;
	const double d = dh_params_.d;

	trans_ << ct, -st * ca, st * sa, a * ct,
	  st, ct * ca, -ct * sa, a * st,
	  0, sa, ca, d,
	  0, 0, 0, 1;

	dh_params_changed_ = false;	
      }
      
      return trans_;
    }

    Eigen::VectorXd basicJacobian(Eigen::Matrix4d trans_prev, Eigen::Vector3d ee_pos) const
    {
      Eigen::Vector3d pos_prev = trans_prev.block(0, 3, 3, 1);
      Eigen::Vector3d z_axis_prev = trans_prev.block(0, 2, 3, 1);

      Eigen::VectorXd basic_jacobian = Eigen::VectorXd::Zero(6);
      basic_jacobian.block(0, 0, 3, 1) = z_axis_prev.cross(ee_pos - pos_prev);   // basic_jacobian.block(0, 3) = z_axis_prev.cross(ee_pos - pos_prev);
      basic_jacobian.block(3, 0, 3, 1) = z_axis_prev;   //       basic_jacobian.block(3, 3) = z_axis_prev;
      return basic_jacobian;
    }

    DHParams getDHParams() const { return dh_params_; }
    void setJointAngle(double joint_angle) {
      dh_params_.theta = joint_angle;
      dh_params_changed_ = true;
    }
    void updateJointAngle(double joint_angle) {
      dh_params_.theta += joint_angle;
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
