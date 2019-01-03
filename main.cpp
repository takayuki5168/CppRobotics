#include <array>
#include <vector>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/joint_type.hpp"
#include "robotics/manipulation/links.hpp"
//#include "robotics/core/core.hpp"

int main()
{
  using namespace Robotics;
  const double PI = 3.1416;

  //init();

  auto links = Links({Link(0, -PI/2, .1, 0, JointType::Rotational),
	Link(PI/2,  PI/2, 0, 0, JointType::Rotational),
	Link(0, -PI/2, 0, .4, JointType::Rotational),
	Link(0, PI/2, 0, 0, JointType::Rotational),
	Link(0, -PI/2, 0, .321, JointType::Rotational),
	Link(0, PI/2, 0, 0, JointType::Rotational),
	Link(0, 0, 0, 0, JointType::Rotational)});
  
  //gl::draw(links);
  
  auto pos = links.calcForwardKinematics();
  std::cout << pos << std::endl;
  
  return 0;
}
