#pragma once


#include <vector>
#include <osqp.h>
#include "robotics/util/csc_matrix.hpp"
#include "robotics/util/util.hpp"

namespace Robotics
{
  namespace Util
  {
    Eigen::VectorXd qp(Eigen::MatrixXd P_mat, Eigen::VectorXd q_vec,
		       Eigen::MatrixXd A_mat, Eigen::VectorXd l_vec, Eigen::VectorXd u_vec){

      // P
      CSCMatrix P_csc_mat = eigenMatrixToCSCMatrix(P_mat);
      int P_nnz = P_csc_mat.data.size();
      c_float P_x[P_csc_mat.data.size()];
      for (int i = 0; i < P_csc_mat.data.size(); i++) {
	P_x[i] = P_csc_mat.data.at(i);
      }
      c_int P_i[P_csc_mat.i.size()];
      for (int i = 0; i < P_csc_mat.i.size(); i++) {
	P_i[i] = P_csc_mat.i.at(i);
      }
      c_int P_p[P_csc_mat.p.size()];
      for (int i = 0; i < P_csc_mat.p.size(); i++) {
	P_p[i] = P_csc_mat.p.at(i);
      }
      // q
      c_float q[q_vec.size()];
      for (int i = 0; i < q_vec.size(); i++) {
	q[i] = q_vec(i);
      }

      // A
      CSCMatrix A_csc_mat = eigenMatrixToCSCMatrix(A_mat);
      int A_nnz = A_csc_mat.data.size();      
      c_float A_x[A_csc_mat.data.size()];
      for (int i = 0; i < A_csc_mat.data.size(); i++) {
	A_x[i] = A_csc_mat.data.at(i);
      }
      c_int A_i[A_csc_mat.i.size()];
      for (int i = 0; i < A_csc_mat.i.size(); i++) {
	A_i[i] = A_csc_mat.i.at(i);
      }
      c_int A_p[A_csc_mat.p.size()];
      for (int i = 0; i < A_csc_mat.p.size(); i++) {
	A_p[i] = A_csc_mat.p.at(i);
      }

      // l, u
      c_float l[l_vec.size()];
      for (int i = 0; i < l_vec.size(); i++) {
	l[i] = l_vec(i);
      }
      c_float u[u_vec.size()];
      for (int i = 0; i < u_vec.size(); i++) {
	u[i] = u_vec(i);
      }

      // init OSQP variables
      OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
      OSQPWorkspace *work;
      OSQPData *data;
      data = (OSQPData *)c_malloc(sizeof(OSQPData));
      data->n = P_mat.row(0).size();
      data->m = l_vec.size();
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

      //
      Eigen::VectorXd result(q_vec.size());
      if (std::string(work->info->status) == "solved") {// or std::string(work->info->status) == "dual infeasible") {
	for (int i = 0; i < q_vec.size(); i++) {
	  result(i) = work->solution->x[i];
	}
      } else {
	for (int i = 0; i < q_vec.size(); i++) {
	  result(i) = 0.;
	}
      }

      // Clean workspace
      osqp_cleanup(work);
      c_free(data->A);
      c_free(data->P);
      c_free(data);
      c_free(settings);

      return result;
    }
  }   // namespace Util
}   // namespace Robotics
