#include <array>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/joint_type.hpp"
#include "robotics/manipulation/links.hpp"
#include "robotics/core/core.hpp"
#include "robotics/core/constant.hpp"
#include "robotics/math/qp.hpp"

#include <qpOASES.hpp>


/*
int main( )
{
  USING_NAMESPACE_QPOASES

    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
  real_t A[1*2] = { 1.0, 1.0 };
  real_t g[2] = { 1.5, 1.0 };
  real_t lb[2] = { 0.5, -2.0 };
  real_t ub[2] = { 5.0, 2.0 };
  real_t lbA[1] = { -1.0 };
  real_t ubA[1] = { 2.0 };


  QProblem example( 2,1 );

  int nWSR = 10;
  example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

  real_t xOpt[2];
  example.getPrimalSolution( xOpt );
  printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );

  //example.printOptions();

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
    
    auto ee_pose = links.forwardKinematics();
    std::cout << "actual pose"
	      << " (" << ee_pose(0) << ", " << ee_pose(1) << ", " << ee_pose(2)
	      << ", " << ee_pose(3) << ", " << ee_pose(4) << ", " << ee_pose(5) << ")" << std::endl;
  }();

  [&](){   // Inverse Kinematics
    return;
    std::cout << "[Inverse Kinematics]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    std::cout << "reference pose"
	      << " (" << ref_pose(0) << ", " << ref_pose(1) << ", " << ref_pose(2)
	      << ", " << ref_pose(3) << ", " << ref_pose(4) << ", " << ref_pose(5) << ")" << std::endl;
    links.inverseKinematics(ref_pose);

    auto ee_pose = links.forwardKinematics();
    std::cout << "actual pose"
	      << " (" << ee_pose(0) << ", " << ee_pose(1) << ", " << ee_pose(2)
	      << ", " << ee_pose(3) << ", " << ee_pose(4) << ", " << ee_pose(5) << ")" << std::endl;
  }();

  [&](){   // Inverse Kinematics with QP
    //return;
    std::cout << "[Inverse Kinematics with QP]" << std::endl;
    links.initPose();

    Eigen::VectorXd ref_pose(6);
    ref_pose << 0.4, 0, 0, 0, 1.57, 0;

    std::cout << "reference pose"
	      << " (" << ref_pose(0) << ", " << ref_pose(1) << ", " << ref_pose(2)
	      << ", " << ref_pose(3) << ", " << ref_pose(4) << ", " << ref_pose(5) << ")" << std::endl;
    links.inverseKinematicsWithQP(ref_pose);

    auto ee_pose = links.forwardKinematics();
    std::cout << "actual pose"
	      << " (" << ee_pose(0) << ", " << ee_pose(1) << ", " << ee_pose(2)
	      << ", " << ee_pose(3) << ", " << ee_pose(4) << ", " << ee_pose(5) << ")" << std::endl;
  }();


  //while (true) {}
  
  return 0;
}
