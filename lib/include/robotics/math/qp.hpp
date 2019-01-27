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

    QProblem example(2,1);

    Options options;
    example.setOptions(options);
    
    int_t nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR);

    real_t xOpt[2];
    example.getPrimalSolution(xOpt);

    std::vector<double> result;
    for (int i = 0; i < 10; i++) {
      result.push_back(xOpt[i]);
    }
    return result;
  }

  void po(std::vector<double> a) {}
}   // namespace Robotics
