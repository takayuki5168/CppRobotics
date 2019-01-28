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

  Eigen::VectorXd vectorToEigenVector(std::vector<double> vector)
  {
    Eigen::VectorXd eigen_vector(vector.size());
    for (int i = 0; i < vector.size(); i++) {
      eigen_vector(i) = vector.at(i);
    }
    return eigen_vector;
  }
}   // end namespace Robotics
