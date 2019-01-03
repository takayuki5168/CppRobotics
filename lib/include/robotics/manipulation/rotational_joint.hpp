#pragma once

#include "robotics/manipulation/abst_joint.hpp"

namespace Robotics
{
  class RotationalJoint : public AbstJoint
  {
  public:
    RotationalJoint(double dh_a, double dh_alpha, double dh_d,
		    double init_joint_angle)
      : AbstJoint(dh_a, dh_alpha, dh_d, init_joint_angle)
    {}

    Eigen::Matrix4d calcTransformationMatrix(double joint_angle) const
    {
      AbstJoint::calcTransformationMatrix(dh_params_.a, dh_params_.alpha, dh_params_.d, joint_angle);
    }

    void setJointAngle(double joint_angle) { dh_params_.theta = joint_angle; }
    double getJointAngle(double joint_angle) const { return dh_params_.theta; }
  private:
  };
} // end namespace Robotics
