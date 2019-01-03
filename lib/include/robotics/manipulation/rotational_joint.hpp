#pragma once

#include "robotics/manipulation/abst_joint.hpp"

namespace Robotics
{
  class RotationalJoint : public AbstJoint
  {
  public:
    RotationalJoint(double init_joint_angle, double dh_alpha, double dh_a, double dh_d)
      : AbstJoint(init_joint_angle, dh_alpha, dh_a, dh_d)
    {}

    Eigen::Matrix4d calcTransformationMatrix(double joint_angle) const
    {
      AbstJoint::calcTransformationMatrix(joint_angle, dh_params_.alpha, dh_params_.a, dh_params_.d);
    }

    void setJointAngle(double joint_angle) { dh_params_.theta = joint_angle; }
    double getJointAngle(double joint_angle) const { return dh_params_.theta; }
  private:
  };
} // end namespace Robotics
