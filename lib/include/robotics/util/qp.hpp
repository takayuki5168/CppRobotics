#pragma once

#include <vector>
#include <qpOASES.hpp>

namespace Robotics
{
  namespace Util
  {
    USING_NAMESPACE_QPOASES
    Eigen::VectorXd qp(Eigen::MatrixXd H_mat, Eigen::VectorXd g_mat, Eigen::VectorXd lb_mat, Eigen::VectorXd ub_mat)
    {
      real_t H[H_mat.size()];
      real_t g[g_mat.size()];
      real_t lb[lb_mat.size()];
      real_t ub[ub_mat.size()];

      for (int i = 0; i < H_mat.size(); i++) { H[i] = H_mat(i); }
      for (int i = 0; i < g_mat.size(); i++) { g[i] = g_mat[i]; }
      for (int i = 0; i < lb_mat.size(); i++) { lb[i] = lb_mat[i]; }
      for (int i = 0; i < ub_mat.size(); i++) { ub[i] = ub_mat[i]; }

      QProblemB example(g_mat.size());

      real_t xOpt[g_mat.size()];

      //int_t nWSR = 10;
      //example.init(H, g, lb, ub, nWSR);
      
      Options options;
      options.printLevel = PL_NONE;
      example.setOptions(options);
      int_t nWSR = 10;      
      getSimpleStatus(example.init(H, g, lb, ub, nWSR), BT_FALSE);

      
      example.getPrimalSolution(xOpt);

      Eigen::VectorXd result(g_mat.size());
      //std::cout << example.getObjVal() << std::endl;
      for (int i = 0; i < g_mat.size(); i++) {
	//std::cout << xOpt[i] << " ";
	result[i] = xOpt[i];
      }
      //std::cout << std::endl;
      return result;
    }

    Eigen::VectorXd qp(Eigen::MatrixXd H_mat, Eigen::VectorXd g_mat,
		       Eigen::VectorXd lb_mat, Eigen::VectorXd ub_mat,
		       Eigen::MatrixXd A_mat, Eigen::VectorXd lbA_mat, Eigen::VectorXd ubA_mat)
    {
      real_t H[H_mat.size()];
      real_t g[g_mat.size()];
      real_t lb[lb_mat.size()];
      real_t ub[ub_mat.size()];
      real_t A[A_mat.size()];
      real_t lbA[lbA_mat.size()];
      real_t ubA[ubA_mat.size()];

      for (int i = 0; i < H_mat.size(); i++) { H[i] = H_mat(i); }
      for (int i = 0; i < g_mat.size(); i++) { g[i] = g_mat[i]; }
      for (int i = 0; i < lb_mat.size(); i++) { lb[i] = lb_mat[i]; }
      for (int i = 0; i < ub_mat.size(); i++) { ub[i] = ub_mat[i]; }
      for (int i = 0; i < A_mat.size(); i++) { A[i] = A_mat(i); }
      for (int i = 0; i < lbA_mat.size(); i++) { lbA[i] = lbA_mat[i]; }
      for (int i = 0; i < ubA_mat.size(); i++) { ubA[i] = ubA_mat[i]; }

      //QProblem example(g_mat.size(), lb_mat.size() + lbA_mat.size());   // TODO
      QProblemB example(g_mat.size());

      int_t nWSR = 10;
      //example.init(H, g, A, lb, ub, lbA, ubA, nWSR);
      example.init(H, g, lb, ub, nWSR);    

      real_t xOpt[g_mat.size()];
      example.getPrimalSolution(xOpt);
      //example.getDualSolution(xOpt);
      //std::cout << sizeof(xOpt) << " " << sizeof(real_t) << std::endl;

      Eigen::VectorXd result(g_mat.size());    
      for (int i = 0; i < g_mat.size(); i++) {
	//std::cout << xOpt[i] << " ";
	result[i] = xOpt[i];
      }
      //std::cout << std::endl;
      return result;
    }
  } // namespace Util
}   // namespace Robotics
