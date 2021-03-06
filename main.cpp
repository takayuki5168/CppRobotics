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
#include "robotics/util/util.hpp"
#include "robotics/util/qp.hpp"
#include <osqp.h>

/*
int main(int argc, char **argv) {
  using namespace Robotics;
  Eigen::Matrix2d P;
  P << 4, 1, 1, 2;
  Eigen::Vector2d q;
  q << 1, 1;
  Eigen::MatrixXd A(3, 2);
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Vector3d l;
  l << 1, 0, 0;
  Eigen::Vector3d u;
  u << 1, 0.7, 0.7;
  
  std::cout << Util::qp(P, q, A, l, u) << std::endl;
  return 0;
}
*/
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
    //return;
    std::cout << "[Inverse Kinematics]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    links.inverseKinematics(ref_pose, true, 0.001);
  }();

  [&](){   // Inverse Kinematics with SQP, using OSQP
    return;
    std::cout << "[Inverse Kinematics with SQP, using OSQP]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    links.inverseKinematicsWithSQP(ref_pose, true, 0.001, 0);
  }();

  [&](){   // Inverse Kinematics with SQP, using qpOASES
    //return;
    std::cout << "[Inverse Kinematics with SQP, using qpOASES]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    links.inverseKinematicsWithSQP(ref_pose, true, 0.001, 1);
  }();
  

  //while (true) {}
  
  return 0;
}
