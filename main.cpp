#include <array>
#include <vector>
#include "robotics/manipulation/links.hpp"

int main()
{
  using namespace Robotics;

  Links links = Links(2,
		      std::vector<double>{1, 1},   // dh_a
		      std::vector<double>{1, 1},   // dh_alpha
		      std::vector<double>{1, 1},   // dh_d
		      std::vector<double>{1, 1},   // dh_theta
		      std::vector<double>{1, 1},
		      std::vector<double>{1, 1});
  links.calcTransformationMatrix();
  
  return 0;
}
