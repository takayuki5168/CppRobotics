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
#include <osqp.h>

int main(int argc, char **argv) {
  // Load problem data
  c_float P_x[4] =
  { 4.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
    2.00000000000000000000, };
  c_int   P_nnz  = 4;
  c_int   P_i[4] = { 0, 1, 0, 1, };
  c_int   P_p[3] = { 0, 2, 4, };
  c_float q[2]   = { 1.00000000000000000000, 1.00000000000000000000, };
  c_float A_x[4] =
  { 1.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
    1.00000000000000000000, };
  c_int   A_nnz  = 4;
  c_int   A_i[4] = { 0, 1, 0, 2, };
  c_int   A_p[3] = { 0, 2, 4, };
  c_float l[3]   =
  { 1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, };
  c_float u[3] =
  { 1.00000000000000000000, 0.69999999999999995559, 0.69999999999999995559, };
  c_int n = 2;
  c_int m = 3;


  // Problem settings
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

  // Structures
  OSQPWorkspace *work; // Workspace
  OSQPData *data;      // OSQPData

  // Populate data
  data    = (OSQPData *)c_malloc(sizeof(OSQPData));
  data->n = n;
  data->m = m;
  data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
  data->l = l;
  data->u = u;


  // Define Solver settings as default
  osqp_set_default_settings(settings);

  // Setup workspace
  work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  // Clean workspace
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

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
*/
