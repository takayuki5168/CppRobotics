#include <array>
#include <vector>
#include "robotics/manipulation/links.hpp"
#include "robotics/manipulation/rotational_joint.hpp"

int main()
{
  using namespace Robotics;
  const double PI = 3.1416;

  auto links = Links({RotationalJoint(0, -PI/2, .1, 0),
	RotationalJoint(PI/2,  PI/2, 0, 0),
	RotationalJoint(0, -PI/2, 0, .4),
	RotationalJoint(0, PI/2, 0, 0),
	RotationalJoint(0, -PI/2, 0, .321),
	RotationalJoint(0, PI/2, 0, 0),
	RotationalJoint(0, 0, 0, 0)});
  
  links.calcForwardKinematics();
  
  return 0;
}
