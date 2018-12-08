#pragma once

namespace CppRobotics
{
  
class NLinkArm
{
public:
  NLinkArm(int link_num, std::vector<double> link_lengths, std::vector<double> min_joint_angles, std::vector<double> max_joint_angles)
    : m_link_num(link_num), m_link_lengths(link_lengths), m_min_joint_angles(min_joint_angles), m_max_joint_angles(max_joint_angles)
  {}

  void calcForwardKinematics()
  {
  }
  
  void calcInverseKinematics()
  {
  }

private:
  std::vector<double> getJointAngles const () { return m_joint_angles; }

  int m_link_num;
  std::vector<double> link_lengths
  std::vector<double> m_joint_angles;
  std::vector<double> m_min_joint_angles;
  std::vector<double> m_max_joint_angles;

  Eigen::MatrixXd m_jacobian;
};
  
} // end namespace NLinkArm
