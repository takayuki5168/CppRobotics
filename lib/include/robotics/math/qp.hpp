#pragma once

#include <vector>
#include <qpOASES.hpp>

namespace Robotics
{
USING_NAMESPACE_QPOASES

std::vector<double> qp(std::vector<double> H_vec, std::vector<double> g_vec,
		       std::vector<double> lb_vec, std::vector<double> ub_vec,
		       std::vector<double> A_vec, std::vector<double> lbA_vec, std::vector<double> ubA_vec)
  {
    real_t H[H_vec.size()];
    real_t g[g_vec.size()];
    real_t lb[lb_vec.size()];
    real_t ub[ub_vec.size()];
    real_t A[A_vec.size()];
    real_t lbA[lbA_vec.size()];
    real_t ubA[ubA_vec.size()];

    for (int i = 0; i < H_vec.size(); i++) { H[i] = H_vec[i]; }
    for (int i = 0; i < g_vec.size(); i++) { g[i] = g_vec[i]; }
    for (int i = 0; i < lb_vec.size(); i++) { lb[i] = lb_vec[i]; }
    for (int i = 0; i < ub_vec.size(); i++) { ub[i] = ub_vec[i]; }
    for (int i = 0; i < A_vec.size(); i++) { A[i] = A_vec[i]; }
    for (int i = 0; i < lbA_vec.size(); i++) { lbA[i] = lbA_vec[i]; }
    for (int i = 0; i < ubA_vec.size(); i++) { ubA[i] = ubA_vec[i]; }

    QProblem example(g_vec.size(), 1);

    Options options;
    example.setOptions(options);
    
    int_t nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR);

    real_t xOpt[g_vec.size()];
    example.getPrimalSolution(xOpt);

    std::vector<double> result;
    for (int i = 0; i < g_vec.size(); i++) {
      result.push_back(xOpt[i]);
    }
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

    QProblem example(3, 1);   // TODO

    int_t nWSR = 10;
    example.init(H,g,A,lb,ub,lbA,ubA, nWSR);

    real_t xOpt[g_mat.size()];
    example.getPrimalSolution(xOpt);

    Eigen::VectorXd result(g_mat.size());
    for (int i = 0; i < g_mat.size(); i++) {
      result[i] = xOpt[i];
    }
    return result;
  }
}   // namespace Robotics
