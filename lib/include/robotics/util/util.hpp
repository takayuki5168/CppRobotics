#pragma once

#include <Eigen/Geometry>
#include <iostream>

namespace Robotics
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
}   // end namespace Robotics
