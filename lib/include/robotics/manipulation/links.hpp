#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/QR>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/orientation_type.hpp"
#include "robotics/core/constant.hpp"
#include "robotics/math/qp.hpp"

namespace Robotics
{
  class Links
  {
  public:
    Links(std::vector<Link> links)
      : link_num_(links.size()), links_(links)
    {}

    void initPose()
    {
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	links_.at(link_idx).setJointAngle(0);
      }
    }

    /*
     * TODO trans_vectorを保持
     *      dh_params_changedを見て計算するかいなか変える
     */
    Eigen::Matrix4d transformationMatrix()
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	trans = trans * links_.at(link_idx).transformationMatrix();
      }
      return trans;
    }

    Eigen::Vector3d eulerAngle()
    {
      Eigen::Matrix4d trans = transformationMatrix();

      double alpha = std::atan2(trans(1, 2), trans(0, 2));
      if (not (-PI / 2 <= alpha and alpha < PI / 2)) {
	alpha = std::atan2(trans(1, 2), trans(0, 2));
      } else if (not (-PI / 2 <= alpha and alpha < PI / 2)) {
	alpha = std::atan2(trans(1, 2), trans(0, 2));	
      }
      double beta = std::atan2(trans(0, 2) * std::cos(alpha) + trans(1, 2) * std::sin(alpha), trans(2, 2));
      double gamma = std::atan2(-trans(0, 0) * std::sin(alpha) + trans(1, 0) * std::cos(alpha), -trans(0, 1) * std::sin(alpha) + trans(1, 1) * std::cos(alpha));

      return Eigen::Vector3d{alpha, beta, gamma};
    }

    Eigen::VectorXd forwardKinematics()
    {
      Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
      Eigen::Matrix4d trans = transformationMatrix();
	
      pose.block(0, 0, 3, 1) = trans.block(0, 3, 3, 1);
      pose.block(3, 0, 3, 1) = eulerAngle();
      
      return pose;
    }


    Eigen::MatrixXd basicJacobian()
    {
      Eigen::Vector3d ee_pos = transformationMatrix().block(0, 3, 3, 1);
      Eigen::MatrixXd basic_jacobian = Eigen::MatrixXd::Zero(6, link_num_);

      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();      
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	basic_jacobian.block(0, link_idx, 6, 1) = links_.at(link_idx).basicJacobian(trans, ee_pos);
	trans *= links_.at(link_idx).transformationMatrix();	
      }

      return basic_jacobian;
    }

    void inverseKinematics(Eigen::VectorXd ref_ee_pose)
    {
      for (int cnt = 0; cnt < 100; cnt++) {
	Eigen::VectorXd ee_pose = forwardKinematics();
	Eigen::VectorXd diff_pose = ref_ee_pose - ee_pose;

	Eigen::MatrixXd basic_jacobian = basicJacobian();

	/*
	// TODO not use eulerAngle(), but use ee_pose
	Eigen::Vector3d euler_angle = eulerAngle();
	double alpha = euler_angle(0, 0);
	double beta = euler_angle(1, 0);
	double gamma = euler_angle(2, 0);

	Eigen::Matrix3d K_zyz;
	K_zyz << 0, -std::sin(alpha), std::cos(alpha) * std::sin(beta),
	  0, std::cos(alpha), std::sin(alpha) * std::sin(beta),
	  1, 0, std::cos(beta);
	
	Eigen::MatrixXd K_alpha = Eigen::MatrixXd::Identity(6, 6);
	K_alpha.block(3, 3, 3, 3) = K_zyz;
	*/


	//Eigen::MatrixXd basic_jacobian_pinv = basic_jacobian.completeOrthogonalDecomposition().pseudoInverse();
	Eigen::MatrixXd basic_jacobian_pinv = basic_jacobian.transpose() * (basic_jacobian * basic_jacobian.transpose() + 0.01 * Eigen::MatrixXd::Identity(6, 6)).inverse();
	
	Eigen::VectorXd diff_theta = basic_jacobian_pinv * K_alpha * diff_pose;
	for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	  links_.at(link_idx).updateJointAngle(diff_theta.block(link_idx, 0, 1, 1)(0, 0));
	}
      }
    }

    void inverseKinematicsWithQP(Eigen::VectorXd ref_ee_pose)
    {
      for (int cnt = 0; cnt < 100; cnt++) {
	Eigen::VectorXd ee_pose = forwardKinematics();
	Eigen::VectorXd diff_pose = ref_ee_pose - ee_pose;

	Eigen::MatrixXd basic_jacobian = basicJacobian();

	/*
	// TODO not use eulerAngle(), but use ee_pose	
	Eigen::Vector3d euler_angle = eulerAngle();
	double alpha = euler_angle(0, 0);
	double beta = euler_angle(1, 0);
	double gamma = euler_angle(2, 0);

	Eigen::Matrix3d K_zyz;
	K_zyz << 0, -std::sin(alpha), std::cos(alpha) * std::sin(beta),
	  0, std::cos(alpha), std::sin(alpha) * std::sin(beta),
	  1, 0, std::cos(beta);
	
	Eigen::MatrixXd K_alpha = Eigen::MatrixXd::Identity(6, 6);
	K_alpha.block(3, 3, 3, 3) = K_zyz;
	*/

	// calculate jacobian
	Eigen::MatrixXd jacobian = K_alpha.inverse() * basic_jacobian;
	Eigen::MatrixXd H = basic_jacobian.transpose() * basic_jacobian * 2;//jacobian.transpose() * jacobian * 2; // TODO is it right?
	std::cout << K_alpha * basic_jacobian << std::endl;
	Eigen::VectorXd g = -2 * (diff_pose.transpose() * K_alpha * basic_jacobian).transpose();//-2 * (jacobian.transpose() * diff_pose).transpose();

	Eigen::VectorXd lb(link_num_);
	for (int i = 0; i < lb.size(); i++) { lb[i] = -100; }
	Eigen::VectorXd ub(link_num_);
	for (int i = 0; i < ub.size(); i++) { ub[i] = 100; }
	
	Eigen::VectorXd diff_theta = qp(H, g, lb, ub);
	for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	  links_.at(link_idx).updateJointAngle(diff_theta.block(link_idx, 0, 1, 1)(0, 0));
	}
      }
    }

  private:
    const int link_num_;
    std::vector<Link> links_;
  };
} // end namespace Robotics
