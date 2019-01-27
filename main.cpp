#include <array>
#include <vector>
#include <Eigen/Geometry>
#include "robotics/manipulation/link.hpp"
#include "robotics/manipulation/joint_type.hpp"
#include "robotics/manipulation/links.hpp"
#include "robotics/core/core.hpp"
#include "robotics/core/constant.hpp"

#include <qpOASES.hpp>

int main( )
{
  USING_NAMESPACE_QPOASES

    /* Setup data of first QP. */
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
  real_t A[1*2] = { 1.0, 1.0 };
  real_t g[2] = { 1.5, 1.0 };
  real_t lb[2] = { 0.5, -2.0 };
  real_t ub[2] = { 5.0, 2.0 };
  real_t lbA[1] = { -1.0 };
  real_t ubA[1] = { 2.0 };

  /* Setup data of second QP. */
  real_t g_new[2] = { 1.0, 1.5 };
  real_t lb_new[2] = { 0.0, -1.0 };
  real_t ub_new[2] = { 5.0, -0.5 };
  real_t lbA_new[1] = { -2.0 };
  real_t ubA_new[1] = { 1.0 };


  /* Setting up QProblem object. */
  QProblem example( 2,1 );

  Options options;
  example.setOptions( options );

  /* Solve first QP. */
  int_t nWSR = 10;
  example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

  /* Get and print solution of first QP. */
  real_t xOpt[2];
  real_t yOpt[2+1];
  example.getPrimalSolution( xOpt );
  example.getDualSolution( yOpt );
  printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
	  xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

  /* Solve second QP. */
  nWSR = 10;
  example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

  /* Get and print solution of second QP. */
  example.getPrimalSolution( xOpt );
  example.getDualSolution( yOpt );
  printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
	  xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

  example.printOptions();
  /*example.printProperties();*/

  /*getGlobalMessageHandler()->listAllMessages();*/

  return 0;
}
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
*/
