#include <cpp_robotics/manipulation/n_link_arm.hpp>

int main()
{
  NLinkArm n_link_arm();

  // ForwardKinematics
  n_link_arm.calc_forward_kinematics();
    
  // InverseKinematics
  n_link_arm.calc_inverse_kinematics();

  // InverseKinematics with NullSpace
  n_link_arm.calc_inverse_kinematics();


  // ForwardDynamics
  n_link_arm.calc_forward_dynamics();

  // InverseDynamics
  n_link_arm.calc_inverse_dynamics();
  
  return 0;
}
