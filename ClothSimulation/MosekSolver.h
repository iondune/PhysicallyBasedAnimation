
#pragma once

#include <ionCore.h>
#include "SSparseMatrix.h"


class MosekSolver
{

public:

	static Eigen::VectorXd Solve(SSparseMatrix const & Q0, Eigen::VectorXd const & c, vector<vector<double>> const & A);

private:

	static void * Environment;

	static void Init();

};
