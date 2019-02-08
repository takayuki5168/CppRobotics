#pragma once

#include <Eigen/Geometry>
#include <iostream>

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

    Eigen::Vector3d rotationMatricesToAngularVelocity(Eigen::Matrix3d rot, Eigen::Matrix3d ref_rot)
    {
      Eigen::Matrix3d res_rot = ref_rot * rot.transpose();
      Eigen::Vector3d l;
      l << res_rot(2, 1) - res_rot(1, 2), res_rot(0, 2) - res_rot(2, 0), res_rot(1, 0) - res_rot(0, 1);
      
      return std::atan2(l.norm(), res_rot(0, 0) + res_rot(1, 1) + res_rot(2, 2) - 1) / l.norm() * l;
    }
  }   // namespace Util
}   // namespace Robotics
