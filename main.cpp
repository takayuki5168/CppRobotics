#include <array>
#include <vector>
#include <unordered_map>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/joint_type.hpp"
#include "robotics/manipulation/links.hpp"
#include "robotics/core/core.hpp"
#include "robotics/core/constant.hpp"
#include "robotics/core/interpreter.hpp"

/*
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

  Interpreter interpreter(links.commands);
  interpreter.run();

  // std::unordered_map<std::string, std::function<std::string(std::vector<double>)>> commands = links.commands;
  // commands.at("inverseKinematics")({1, 1, 1, 1, 1, 1});
  // commands.at("forwardKinematics")({});
  // commands.at("initPose")({});
  // commands.at("forwardKinematics")({});

  return 0;
}
*/


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
    std::cout << "[Forward Kinematics]" << std::endl;
    links.initPose();
    
    auto ee_pose = links.forwardKinematics(true);
  }();

  [&](){   // Inverse Kinematics
    return;
    std::cout << "[Inverse Kinematics]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    links.inverseKinematics(ref_pose, true);
  }();

  [&](){   // Inverse Kinematics with SQP
    //return;
    std::cout << "[Inverse Kinematics with SQP]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    links.inverseKinematicsWithSQP(ref_pose, true);
  }();


  //while (true) {}
  
  return 0;
}
