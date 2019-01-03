#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Geometry>

namespace Robotics
{
  
class Links
{
public:
  Links(int link_num,
	std::vector<double> dh_a, std::vector<double> dh_alpha, std::vector<double> dh_d, std::vector<double> dh_theta,
	std::vector<double> min_joint_angles, std::vector<double> max_joint_angles)
    : link_num_(link_num),
      dh_a_(dh_a), dh_alpha_(dh_alpha), dh_d_(dh_d), dh_theta_(dh_theta),
      min_joint_angles_(min_joint_angles), max_joint_angles_(max_joint_angles)
  {}

  Eigen::Matrix4d calcTransformationMatrix(int link_idx) const
  {
    double a = dh_a_.at(link_idx);
    double alpha = dh_alpha_.at(link_idx);
    double d = dh_d_.at(link_idx);
    double theta = dh_theta_.at(link_idx);

    Eigen::Matrix4d trans;
    trans << std::cos(theta), -std::sin(theta), 0, a,
      std::cos(alpha) * std::sin(theta), std::cos(alpha) * std::cos(theta), -std::sin(alpha), -d * std::sin(alpha),
      std::sin(alpha) * std::sin(theta), std::sin(alpha) * std::cos(theta), std::cos(alpha), d * std::cos(alpha),      
      0, 0, 0, 1;

    return trans;
  }

  Eigen::Matrix4d calcTransformationMatrix() const
  {
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(4, 4);
    for (int link_idx = 0; link_idx < link_num_; link_idx++) {
      trans *= calcTransformationMatrix(link_idx);
    }
    
    return trans;
  }
  
  Eigen::Vector3d calcForwardKinematics()
  {
    return (calcTransformationMatrix() * Eigen::Vector4d(0, 0, 0, 1)).block(0, 0, 3, 1);
  }
  
  void calcInverseKinematics()
  {
  }

private:
  std::vector<double> getJointAngles() const { return joint_angles_; }

  int link_num_;
  std::vector<double> dh_a_;
  std::vector<double> dh_alpha_;
  std::vector<double> dh_d_;
  std::vector<double> dh_theta_;  
  std::vector<double> min_joint_angles_;
  std::vector<double> max_joint_angles_;
  
  std::vector<double> joint_angles_;
};
  
} // end namespace Robotics
