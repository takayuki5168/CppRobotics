#include <array>
#include <vector>
#include "robotics/manipulation/links.hpp"
#include "robotics/manipulation/abst_joint.hpp"
#include "robotics/manipulation/rotational_joint.hpp"

int main()
{
  using namespace Robotics;

  auto links = Links({RotationalJoint(1, 1, 1, 0),
	RotationalJoint(1, 1, 1, 0)});
  
  links.calcForwardKinematics();
  
  return 0;
}
