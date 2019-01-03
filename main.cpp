#include <array>
#include <vector>
#include "robotics/manipulation/links.hpp"
#include "robotics/manipulation/abst_joint.hpp"
#include "robotics/manipulation/rotational_joint.hpp"

int main()
{
  using namespace Robotics;

  auto joints = std::vector<AbstJoint>{RotationalJoint(1, 1, 1, 1),
				       RotationalJoint(1, 1, 1, 1)};
  auto links = Links(joints);
  links.calcForwardKinematics();
  
  return 0;
}
