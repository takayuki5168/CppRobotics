#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/orientation_type.hpp"

namespace Robotics
{
  class Links
  {
  public:
    Links(std::vector<Link> links)
      : link_num_(links.size()), links_(links)
    {}

    /*
     * TODO trans_vectorを保持
     *      dh_params_changedを見て計算するかいなか変える
     */
    Eigen::Matrix4d calcTransformationMatrix()
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	trans *= links_.at(link_idx).calcTransformationMatrix();
      }
      return trans;
    }

    Eigen::Matrix4d calcTransformationMatrix(int end_link_idx)
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
      for (int link_idx = 0; link_idx < end_link_idx; link_idx++) {
	trans *= links_.at(link_idx).calcTransformationMatrix();
      }
      return trans;
    }

    Eigen::VectorXd calcForwardKinematics()
    {
      Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
      calcTransformationMatrix();
      
      Eigen::Matrix4d trans = calcTransformationMatrix();
	
      double alpha = std::atan2(trans(1, 2), trans(1, 3));
      double beta = std::atan2(trans(0, 2) * std::cos(alpha) + trans(1, 2) * std::sin(alpha), trans(2, 2));
      double gamma = std::atan2(-trans(0, 0) * std::sin(alpha) + trans(1, 0) * std::cos(alpha), -trans(0, 1) * std::sin(alpha) + trans(1, 1) * std::cos(alpha));

      pose.block(0, 0, 3, 1) = (trans * Eigen::Vector4d(0, 0, 0, 1)).block(0, 0, 3, 1);
      pose.block(3, 0, 3, 1) = Eigen::Vector3d(alpha, beta, gamma);
      
      return pose;
    }


    Eigen::MatrixXd calcBasicJacobian(std::vector<double> joint_angles)
    {
      Eigen::Vector3d end_effector_pos = calcTransformationMatrix().block(0, 3, 3, 1);
      Eigen::MatrixXd basic_jacobian = Eigen::MatrixXd::Zero(6, link_num_);
      
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	Eigen::Matrix4d trans = calcTransformationMatrix(link_idx);	
	basic_jacobian.block(0, link_idx, 6, 1) = links_.at(link_idx).calcBasicJacobian(joint_angles.at(link_idx), trans, end_effector_pos);
      }

      return basic_jacobian;
    }

    Eigen::MatrixXd calcJacobian(std::vector<double> joint_angles)// const
    {
      Eigen::MatrixXd basic_jacobian = calcBasicJacobian(joint_angles);
      
      Eigen::MatrixXd K_alpha = Eigen::MatrixXd::Zero(6, 6);
      Eigen::Vector3d euler_angles = calcForwardKinematics().block(3, 0, 3, 1);
      double alpha = euler_angles(0, 0);
      double beta = euler_angles(1, 0);
      double gamma = euler_angles(2, 0);
      Eigen::MatrixXd K_zyz(3, 3);
      K_zyz << 0, -std::sin(alpha), std::cos(alpha) * std::sin(beta),
	0, std::cos(alpha), std::sin(alpha) * std::sin(beta),
	1, 0, std::cos(beta);

      K_alpha.block(0, 0, 3, 3) = Eigen::Matrix3d::Ones(3, 3);
      K_alpha.block(3, 3, 3, 3) = K_zyz;
      
      Eigen::MatrixXd jacobian = K_alpha * basic_jacobian;

      return jacobian;
    }
  
    void calcInverseKinematics(Eigen::VectorXd ref_pose)
    {
      for (int cnt = 0; cnt < 1000; cnt++) {
	std::vector<double> joint_angles;
	for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	  joint_angles.push_back(links_.at(link_idx).getDHParams().theta);
	}
	
	Eigen::VectorXd now_pose = calcForwardKinematics();
	Eigen::VectorXd diff_pose = ref_pose - now_pose;
	
	Eigen::MatrixXd jacobian = calcJacobian(joint_angles);
	//std::cout << (jacobian * jacobian.transpose()) << std::endl;
	Eigen::VectorXd theta_dot = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse() * diff_pose;

	for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	  std::cout << theta_dot.block(0, link_idx, 1, 1)(0, 0) << std::endl;
	  links_.at(link_idx).setJointAngle(theta_dot.block(0, link_idx, 1, 1)(0, 0));
	}
	std::cout << now_pose << std::endl;
      }
    }

  private:
    const int link_num_;
    std::vector<Link> links_;
  };
} // end namespace Robotics
