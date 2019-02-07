#pragma once

namespace Robotics
{
  namespace Util
  {
    struct CSCMatrix
    {
      CSCMatrix(std::vector<double> data, std::vector<int> i, std::vector<int> p)
	: data(data), i(i), p(p) {}
      CSCMatrix() {}
    
      std::vector<double> data;
      std::vector<int> i, p;
    };
  }   // namespace Util
}   // namspace Robotics


