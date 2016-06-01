
#pragma once

#include <ionCore.h>
#include "SSparseMatrix.h"


class MosekSolver
{

public:

	static Eigen::VectorXd Solve(Eigen::MatrixXd const & Q0, Eigen::VectorXd const & c, Eigen::MatrixXd const & A, Eigen::VectorXd const & b, Eigen::VectorXd const & x0);

private:

	static void * Environment;

	static void Init();

};
