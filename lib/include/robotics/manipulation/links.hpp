#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"

namespace Robotics
{
  
  class Links
  {
  public:
    Links(std::vector<Link> links)
      : link_num_(links.size()), links_(links)
    {}

    Eigen::Matrix4d calcTransformationMatrix(int link_idx) const
    {
      return links_.at(link_idx).calcTransformationMatrix();
    }

    Eigen::Matrix4d calcTransformationMatrix(int link_idx, double dh_val) const
    {
      return links_.at(link_idx).calcTransformationMatrix(dh_val);
    }

    Eigen::Matrix4d calcTransformationMatrix() const
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(4, 4);
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	trans *= calcTransformationMatrix(link_idx);
      }
    
      return trans;
    }

    Eigen::Matrix4d calcTransformationMatrix(std::vector<double> dh_vals) const
    {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(4, 4);
      for (int link_idx = 0; link_idx < link_num_; link_idx++) {
	trans *= calcTransformationMatrix(link_idx, dh_vals.at(link_idx));
      }
    
      return trans;
    }
    
    Eigen::Vector3d calcForwardKinematics()
    {
      // TODO 姿勢は
      //return (calcTransformationMatrix().inverse() * Eigen::Vector4d(0, 0, 0, 1)).block(0, 0, 3, 1);
      return (calcTransformationMatrix() * Eigen::Vector4d(0, 0, 0, 1)).block(0, 0, 3, 1);      
    }
  
    void calcInverseKinematics(Eigen::Vector3d ref_pos)
    {
    }

  private:
    int link_num_;
    std::vector<Link> links_;
  };
  
} // end namespace Robotics
