#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/orientation_type.hpp"
#include "robotics/core/constant.hpp"
#include "robotics/util/qp.hpp"
#include "robotics/util/util.hpp"
#include <chrono>

namespace Robotics
{
  class Links
  {
  public:
    Links(std::vector<Link> links)
      : link_num_(links.size()), links_(links)
    {
      commands["forwardKinematics"] = [this](std::vector<double>){
	forwardKinematics(true);
	return "po";
      };
      commands["inverseKinematics"] = [this](std::vector<double> ref_pose){
	inverseKinematics(Util::vectorToEigenVector(ref_pose), true);
	return "po";
      };
      commands["initPose"] = [this](std::vector<double>){
	initPose();
	return "po";
      };
    }

    std::unordered_map<std::string, std::function<std::string(std::vector<double>)>> commands;
    
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

    Eigen::VectorXd forwardKinematics(bool print_info=false)
    {
      // start measuring time
      std::chrono::system_clock::time_point  start = std::chrono::system_clock::now();

      Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
      Eigen::Matrix4d trans = transformationMatrix();
	
      pose.block(0, 0, 3, 1) = trans.block(0, 3, 3, 1);
      pose.block(3, 0, 3, 1) = eulerAngle();
      
      // finish measuring time
      std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

      // print infomation about IK with SQP
      if (print_info) {
	std::cout << "[Forward Kinamatics Information]" << std::endl;
	std::cout << "  time: " << elapsed << "[ms]" << std::endl;
	Util::printVector(pose, "  actual pose ");
	
      }
      
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

    void inverseKinematics(Eigen::VectorXd ref_ee_pose, bool print_info=false, double epsilon=0.001)
    {
      // start measuring time
      std::chrono::system_clock::time_point  start = std::chrono::system_clock::now();

      // starr caluclation
      for (int cnt = 0; cnt < 100; cnt++) {  // TODO
	Eigen::VectorXd ee_pose = forwardKinematics();
	Eigen::VectorXd diff_pose = ref_ee_pose - ee_pose;
	if (diff_pose.norm() < epsilon) { break; }

	Eigen::Matrix3d diff_rot = Util::eulerAngleToRotationMatrix(ee_pose.segment(3, 3)).transpose() * Util::eulerAngleToRotationMatrix(ref_ee_pose.segment(3, 3));
	Eigen::Vector3d diff_angular_velocity = Util::rotationMatricesToAngularVelocity(Util::eulerAngleToRotationMatrix(ee_pose.segment(3, 3)), diff_rot);

	// calcurate diff twist
	Eigen::VectorXd diff_twist(6);
	diff_twist.segment(0, 3) = diff_pose.segment(0, 3);
	diff_twist.segment(3, 3) = diff_angular_velocity;

	// calcurate basic jacobian
	Eigen::MatrixXd basic_jacobian = basicJacobian();

	// use SR inverse when calculating pseuso inverse matrix of basic_jacobian
	//Eigen::MatrixXd basic_jacobian_pinv = basic_jacobian.completeOrthogonalDecomposition().pseudoInverse();
	Eigen::MatrixXd basic_jacobian_pinv = basic_jacobian.transpose() * (basic_jacobian * basic_jacobian.transpose() + 0.000000001 * Eigen::MatrixXd::Identity(6, 6)).inverse();

	Eigen::VectorXd diff_theta = basic_jacobian_pinv * diff_twist;
	for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	  links_.at(link_idx).updateJointAngle(diff_theta.block(link_idx, 0, 1, 1)(0, 0));
	}
      }

      // finish measuring time
      std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

      // print infomation about IK with SQP
      if (print_info) {
	Eigen::VectorXd ee_pose = forwardKinematics();
	Eigen::VectorXd diff_pose = ref_ee_pose - ee_pose;

	std::cout << "[Inverse Kinamatics Information]" << std::endl;
	std::cout << "  time: " << elapsed << "[ms]" << std::endl;
	Util::printVector(ref_ee_pose, "  reference pose ");
	Util::printVector(ee_pose, "  actual pose ");		
	Util::printVector(diff_pose, "  diff pose ");
	
      }
    }

    void inverseKinematicsWithSQP(Eigen::VectorXd ref_ee_pose, bool print_info=false, double epsilon=0.001)
    {
      // start measuring time
      std::chrono::system_clock::time_point  start = std::chrono::system_clock::now();
      
      for (int cnt = 0; cnt < 100; cnt++) {
	//for (int cnt = 0; cnt < 1; cnt++) {	
	Eigen::VectorXd ee_pose = forwardKinematics();
	Eigen::VectorXd diff_pose = ref_ee_pose - ee_pose;
	//if (diff_pose.norm() < epsilon) { break; }
	
	Eigen::Matrix3d diff_rot = Util::eulerAngleToRotationMatrix(ee_pose.segment(3, 3)).transpose() * Util::eulerAngleToRotationMatrix(ref_ee_pose.segment(3, 3));
	Eigen::Vector3d diff_angular_velocity = Util::rotationMatricesToAngularVelocity(Util::eulerAngleToRotationMatrix(ee_pose.segment(3, 3)), diff_rot);

	// calcurate diff twist
	Eigen::VectorXd diff_twist(6);
	diff_twist.segment(0, 3) = diff_pose.segment(0, 3);
	diff_twist.segment(3, 3) = diff_angular_velocity;

	// calcurate basic jacobian
	Eigen::MatrixXd basic_jacobian = basicJacobian();

	// set to QP
	Eigen::MatrixXd H = basic_jacobian.transpose() * basic_jacobian;
	Eigen::VectorXd g = (-diff_twist.transpose() / 100. * basic_jacobian).transpose();

	Eigen::VectorXd lb(link_num_);
	for (int i = 0; i < lb.size(); i++) { lb[i] = -1000; }
	Eigen::VectorXd ub(link_num_);
	for (int i = 0; i < ub.size(); i++) { ub[i] = 1000; }

	/*
	std::cout << std::endl;
	ee_pose = forwardKinematics();
	diff_pose = ref_ee_pose - ee_pose;
	std::cout << diff_pose.norm() << std::endl;
	*/
	
	Eigen::VectorXd diff_theta = Util::qp(H, g, lb, ub);
	for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	  links_.at(link_idx).updateJointAngle(diff_theta.block(link_idx, 0, 1, 1)(0, 0));
	}
	/*
	ee_pose = forwardKinematics();
	diff_pose = ref_ee_pose - ee_pose;
	std::cout << diff_pose.norm() << std::endl;
	*/
      }
      
      // finish measuring time
      std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

      // print infomation about IK
      if (print_info) {
	Eigen::VectorXd ee_pose = forwardKinematics();
	Eigen::VectorXd diff_pose = ref_ee_pose - ee_pose;
	std::cout << "[Inverse Kinamatics with QP Information]" << std::endl;	
	std::cout << "  time: " << elapsed << "[ms]" << std::endl;
	Util::printVector(ref_ee_pose, "  reference pose ");
	Util::printVector(ee_pose, "  actual pose ");		
	Util::printVector(diff_pose, "  diff pose ");
      }
    }

  private:
    const int link_num_;
    std::vector<Link> links_;
  };
}   // namespace Robotics
