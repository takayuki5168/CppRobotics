#include <array>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/joint_type.hpp"
#include "robotics/manipulation/links.hpp"
#include "robotics/core/core.hpp"
#include "robotics/core/constant.hpp"

int main(int argc, char** argv)
{
  using namespace Robotics;

  //init(&argc, argv, true);   // &argc, argv, with_viewer

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
    
    auto ee_pose = links.forwardKinematics();
    std::cout << ee_pose << std::endl;
  }();

  [&](){   // Inverse Kinematics
    //return;

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;
    std::cout << "Start IK" << std::endl;
    std::cout << ref_pose << std::endl;
    links.inverseKinematics(ref_pose);

    std::cout << "End IK" << std::endl;
    auto ee_pose = links.forwardKinematics();
    std::cout << ee_pose << std::endl;
    
  }();
  

  //while (true) {}
  
  return 0;
}
