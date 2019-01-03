#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/abst_joint.hpp"

namespace Robotics
{
  
  class Links
  {
  public:
    Links(std::vector<AbstJoint> joints)
      : link_num_(joints.size()), joints_(joints)
    {}

    Eigen::Matrix4d calcTransformationMatrix(int link_idx) const
    {
      return joints_.at(link_idx).calcTransformationMatrix();
    }

    Eigen::Matrix4d calcTransformationMatrix(int link_idx, double val) const
    {
      const DHParams& dh_params = joints_.at(link_idx).getDHParams();
      const double dh_a = dh_params.a;
      const double dh_alpha = dh_params.alpha;
      const double dh_d = dh_params.d;
      const double dh_theta = dh_params.theta;

      return joints_.at(link_idx).calcTransformationMatrix(dh_a, dh_alpha, dh_d, dh_theta); // change val one of args
    }

    Eigen::Matrix4d calcTransformationMatrix() const
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(4, 4);
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	trans *= calcTransformationMatrix(link_idx);
      }
    
      return trans;
    }

    Eigen::Matrix4d calcTransformationMatrix(std::vector<double> vals) const
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(4, 4);
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	trans *= calcTransformationMatrix(link_idx, vals.at(link_idx));
      }
    
      return trans;
    }
    
  
    Eigen::Vector3d calcForwardKinematics()
    {
      return (calcTransformationMatrix() * Eigen::Vector4d(0, 0, 0, 1)).block(0, 0, 3, 1);
    }
  
    void calcInverseKinematics(Eigen::Vector3d ref_pos)
    {
    }

  private:
    std::vector<double> getJointAngles() const { return joint_angles_; }

    int link_num_;
    std::vector<AbstJoint> joints_;
  
    std::vector<double> joint_angles_;
  };
  
} // end namespace Robotics
