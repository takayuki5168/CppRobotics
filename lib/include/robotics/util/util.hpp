#pragma once

#include <Eigen/Geometry>
#include <iostream>
#include <osqp.h>
#include <robotics/util/csc_matrix.hpp>

namespace Robotics
{
  namespace Util
  {
    void printVector(Eigen::VectorXd vec, std::string preface="")
    {
      if (preface != "") {
	std::cout << preface << " ";
      }
      std::cout << "(";
      for (int i = 0; i < vec.size(); i++) {
	if (i != vec.size() - 1) {
	  std::cout << vec(i) << ", ";
	} else {
	  std::cout << vec(i) << ")" << std::endl;
	}
      }
    }

    void printMatrix(Eigen::MatrixXd mat, std::string preface="")
    {
      if (preface != "") {
	std::cout << preface << " ";
      }
      std::cout << "(";
      for (int i = 0; i < mat.col(0).size(); i++) {
	std::cout << "(";	
	for (int j = 0; j < mat.row(0).size(); j++) {
	  if (j != mat.row(0).size() - 1) {
	    std::cout << mat(i, j) << ", ";
	  } else {
	    if (i == mat.col(0).size() - 1) {
	      std::cout << mat(i, j) << ")";
	    } else {
	      std::cout << mat(i, j) << ")" << std::endl;
	    }
	  }
	}
      }
      std::cout << ")" << std::endl;
    }
    

    Eigen::VectorXd vectorToEigenVector(std::vector<double> vector)
    {
      Eigen::VectorXd eigen_vector(vector.size());
      for (int i = 0; i < vector.size(); i++) {
	eigen_vector(i) = vector.at(i);
      }
      return eigen_vector;
    }

    Eigen::Matrix3d eulerAngleToRotationMatrix(Eigen::VectorXd euler_angle)
    {
      Eigen::Matrix3d rot;
      const double sa = std::sin(euler_angle(0));
      const double sb = std::sin(euler_angle(1));
      const double sg = std::sin(euler_angle(2));
      const double ca = std::cos(euler_angle(0));
      const double cb = std::cos(euler_angle(1));
      const double cg = std::cos(euler_angle(2));    
    
      rot << ca * cb * cg - sa * sg, -ca * cb * cg - sa * cg, ca * sb,
	sa * cb * cg + ca * sg, -sa * cb * sg + ca * cg, sa * sb,
	-sb * cg, sb * sg, cb;
      return rot;
    }

    Eigen::Matrix4d poseToTransformationMatrix(Eigen::VectorXd pose)
    {
      Eigen::Matrix3d rot = eulerAngleToRotationMatrix(pose.segment(3, 3));
    
      Eigen::Matrix4d trans;
      trans.block(0, 0, 3, 3) = rot;
      trans.block(0, 3, 3, 1) = pose.segment(0, 3);
      trans(3, 3) = 1.;
      return trans;
    }

    /*
    Eigen::Vector3d rotationMatricesToAngularVelocity(Eigen::Matrix3d rot, Eigen::Matrix3d diff_rot)
    {
      Eigen::Matrix3d mat = diff_rot * rot.transpose();
      Eigen::Vector3d angular_velocity;
      angular_velocity << -mat(1, 2), mat(0, 2), -mat(0, 1);
      return angular_velocity;
    }
    */
    Eigen::Vector3d rotationMatricesToAngularVelocity(Eigen::Matrix3d rot, Eigen::Matrix3d rot_ref)
    {
      Eigen::Matrix3d a = (rot_ref * rot.transpose());
      Eigen::Matrix3d mat = (a - a.transpose()) * (rot_ref * rot.transpose());
      Eigen::Vector3d angular_velocity;
      angular_velocity << -mat(1, 2), mat(0, 2), -mat(0, 1);
      return angular_velocity;
    }

    CSCMatrix eigenMatrixToCSCMatrix(Eigen::MatrixXd eigen_mat, double epsilon=0.0001)
    {
      CSCMatrix csc_mat;
      csc_mat.p.push_back(0);

      int cnt = 0;
      for (int i = 0; i < eigen_mat.row(0).size(); i++) {
	for (int j = 0; j < eigen_mat.col(0).size(); j++) {
	  if (std::abs(eigen_mat(j, i) > epsilon)) {
	    cnt += 1;
	    csc_mat.data.push_back(eigen_mat(j, i));
	    csc_mat.i.push_back(j);
	  }
	}
	csc_mat.p.push_back(cnt);
      }

      /*
      for (int i = 0; i < csc_mat.i.size(); i++) {
	std::cout << csc_mat.i.at(i) << " ";
      }

      std::cout << std::endl;
      for (int i = 0; i < csc_mat.p.size(); i++) {
	std::cout << csc_mat.p.at(i) << " ";	
      }

      std::cout << std::endl;
      for (int i = 0; i < csc_mat.data.size(); i++) {
	std::cout << csc_mat.data.at(i) << " ";	
      }
      */
      
      return csc_mat;
    }
  }   // namespace Util
}   // namespace Robotics
