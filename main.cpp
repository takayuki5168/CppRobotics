#include <array>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/joint_type.hpp"
#include "robotics/manipulation/links.hpp"
#include "robotics/core/core.hpp"

int main(int argc, char** argv)
{
  using namespace Robotics;
  const double PI = 3.1416;

  init(&argc, argv, true);   // &argc, argv, with_viewer

  auto links = Links({Link(0, -PI/2, .1, 0, JointType::Rotational),
	Link(PI/2,  PI/2, 0, 0, JointType::Rotational),
	Link(0, -PI/2, 0, .4, JointType::Rotational),
	Link(0, PI/2, 0, 0, JointType::Rotational),
	Link(0, -PI/2, 0, .321, JointType::Rotational),
	Link(0, PI/2, 0, 0, JointType::Rotational),
	Link(0, 0, 0, 0, JointType::Rotational)});
  
  //draw(links);

  [&](){   // Forward Kinematics
    return;
    
    auto pose = links.calcForwardKinematics();
    std::cout << pose << std::endl;
  }();

  [&](){   // Inverse Kinematics
    //return;

    Eigen::VectorXd ref_pose(6);
    ref_pose << -0.4, 0, 0, 3.14, 0, -1.57;
    links.calcInverseKinematics(ref_pose);
  }();
  

  //while (true) {}
  
  return 0;
}
