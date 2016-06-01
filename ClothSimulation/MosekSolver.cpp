
#include "MosekSolver.h"
#include <mosek.h>


void * MosekSolver::Environment = nullptr;

static void MSKAPI printstr(void *handle, MSKCONST char str[])
{
	printf("%s", str);
}

int CountNonZeroSymmetric(Eigen::MatrixXd const & m)
{
	uint NonZero = 0;

	assert(m.rows() == m.cols());
	//assert(IsSymmetric(m));

	for (int i = 0; i < m.rows(); ++ i)
	{
		double ProductSum = 0;
		for (int j = 0; j <= i; ++ j)
		{
			if (m(i, j) != 0)
				NonZero ++;
		}
	}

	return NonZero;
}

Eigen::VectorXd  MosekSolver::Solve(Eigen::MatrixXd const & Q0, Eigen::VectorXd const & c, Eigen::MatrixXd const & A, Eigen::VectorXd const & b, Eigen::VectorXd const & x0)
{
	Eigen::VectorXd ReturnValue;

	if (! Environment)
	{
		Init();
	}

	assert(Q0.rows() == c.size());
	assert(Q0.rows() == Q0.cols());

	int const NUMCON = (int) A.rows();
	int const NUMVAR = (int) Q0.rows();
	int const NUMQNZ = CountNonZeroSymmetric(Q0);

	ReturnValue.resize(NUMVAR);

	//MSKint32t 
	//	aptrb[] = { 0,   1,   2 },
	//	aptre[] = { 1,   2,   3 },
	//	asub[] = { 0,   0,   0 };
	//double        aval[] = { 1.0, 1.0, 1.0 };

	MSKint32t * qsubi = new MSKint32t[NUMQNZ];
	MSKint32t * qsubj = new MSKint32t[NUMQNZ];
	double * qval = new double[NUMQNZ];

	MSKint32t     i, j;
	double * xx = new double[NUMVAR];

	MSKenv_t      env = Environment;
	MSKtask_t     task = NULL;
	MSKrescodee   r;

	assert(A.cols() == NUMVAR);

	/* Create the optimization task. */
	r = MSK_maketask(env, NUMCON, NUMVAR, &task);

	if (r == MSK_RES_OK)
	{
		r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

		/* Append 'NUMCON' empty constraints.
		The constraints will initially have no bounds. */
		if (r == MSK_RES_OK)
			r = MSK_appendcons(task, NUMCON);

		/* Append 'NUMVAR' variables.
		The variables will initially be fixed at zero (x=0). */
		if (r == MSK_RES_OK)
			r = MSK_appendvars(task, NUMVAR);

		if (r == MSK_RES_OK)
		{
			MSKrealt * xc = new MSKrealt[NUMVAR];
			for (i = 0; i < NUMVAR; ++ i)
				xc[i] = x0(i);
			r = MSK_putxc(task, MSK_SOL_BEGIN, xc);
		}

		/* Optionally add a constant term to the objective. */
		if (r == MSK_RES_OK)
			r = MSK_putcfix(task, 0.0);
		for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j)
		{
			/* Set the linear term c_j in the objective.*/
			if (r == MSK_RES_OK)
				r = MSK_putcj(task, j, c(j));

			/* Set the bounds on variable j.
			blx[j] <= x_j <= bux[j] */
			if (r == MSK_RES_OK)
				r = MSK_putvarbound(task,
					j,                  /* Index of variable.*/
					MSK_BK_LO,          /* Bound key.*/
					0.0,      /* Numerical value of lower bound.*/
					+MSK_INFINITY);     /* Numerical value of upper bound.*/

			MSKint32t NumNonZeroA = 0;

			for (i = 0; i < NUMCON; ++ i)
			{
				if (A(i, j) != 0)
				{
					NumNonZeroA ++;
				}
			}

			MSKint32t * RowIndicesA = new MSKint32t[NumNonZeroA];
			MSKrealt * RowValuesA = new MSKrealt[NumNonZeroA];

			uint Index = 0;
			printf("column j=%d of A contains...\n", j);
			for (i = 0; i < NUMCON; ++ i)
			{
				if (A(i, j) != 0)
				{
					RowIndicesA[Index] = i;
					RowValuesA[Index] = A(i, j);
					printf("A(%d, %d) = %.6f\n", i, j, A(i, j));
					Index ++;
				}
			}
			printf("\n");
			assert(Index == NumNonZeroA);

								 /* Input column j of A */
			if (r == MSK_RES_OK)
				r = MSK_putacol(task,
					j,                 /* Variable (column) index.*/
					NumNonZeroA, /* Number of non-zeros in column j.*/
					RowIndicesA,     /* Pointer to row indexes of column j.*/
					RowValuesA);    /* Pointer to Values of column j.*/

		}

		/* Set the bounds on constraints.
		for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
		for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i)
			r = MSK_putconbound(task,
				i,                  /* Index of constraint.*/
				MSK_BK_UP,          /* Bound key.*/
				-MSK_INFINITY, b(i)               /* Numerical value of lower bound.*/
				);// +MSK_INFINITY);     /* Numerical value of upper bound.*/

		if (r == MSK_RES_OK)
		{
			/*
			* The lower triangular part of the Q
			* matrix in the objective is specified.
			*/

			uint NonZeroIndexInQ0 = 0;
			for (i = 0; i < Q0.rows(); ++ i)
			{
				double ProductSum = 0;
				for (j = 0; j <= i; ++ j)
				{
					double const Value = Q0(i, j);
					if (Value != 0)
					{
						qsubi[NonZeroIndexInQ0] = i;   qsubj[NonZeroIndexInQ0] = j;  qval[NonZeroIndexInQ0] = Value;
						printf("Q0(%d, %d) = %.6f\n", i, j, Value);
						NonZeroIndexInQ0 ++;
					}
				}
			}
			printf("\n");
			assert(NonZeroIndexInQ0 == NUMQNZ);

			/* Input the Q for the objective. */
			r = MSK_putqobj(task, NUMQNZ, qsubi, qsubj, qval);
		}

		if (r == MSK_RES_OK)
		{
			MSKrescodee trmcode;

			/* Run optimizer */
			r = MSK_optimizetrm(task, &trmcode);

			/* Print a summary containing information
			about the solution for debugging purposes*/
			MSK_solutionsummary(task, MSK_STREAM_MSG);

			if (r == MSK_RES_OK)
			{
				MSKsolstae solsta;
				int j;

				MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

				switch (solsta)
				{
				case MSK_SOL_STA_OPTIMAL:
				case MSK_SOL_STA_NEAR_OPTIMAL:
					MSK_getxx(task,
						MSK_SOL_ITR,    /* Request the interior solution. */
						xx);

					printf("Optimal primal solution\n");
					for (j = 0; j < NUMVAR; ++j)
					{
						printf("x[%d]: %e\n", j, xx[j]);
						ReturnValue(j) = xx[j];
					}

					break;
				case MSK_SOL_STA_DUAL_INFEAS_CER:
				case MSK_SOL_STA_PRIM_INFEAS_CER:
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
					printf("Primal or dual infeasibility certificate found.\n");
					break;

				case MSK_SOL_STA_UNKNOWN:
					printf("The status of the solution could not be determined.\n");
					break;
				default:
					printf("Other solution status.");
					break;
				}
			}
			else
			{
				printf("Error while optimizing.\n");
			}
		}

		if (r != MSK_RES_OK)
		{
			/* In case of an error print error code and description. */
			char symname[MSK_MAX_STR_LEN];
			char desc[MSK_MAX_STR_LEN];

			printf("An error occurred while optimizing.\n");
			MSK_getcodedesc(r,
				symname,
				desc);
			printf("Error %s - '%s'\n", symname, desc);
		}
	}

	MSK_deletetask(&task);

	delete[] qsubi;
	delete[] qsubj;
	delete[] qval;
	delete[] xx;

	return ReturnValue;
}

void MosekSolver::Init()
{
	MSKrescodee r = MSK_makeenv(&Environment, nullptr);

	if (r != MSK_RES_OK)
	{
		/* In case of an error print error code and description. */
		char symname[MSK_MAX_STR_LEN];
		char desc[MSK_MAX_STR_LEN];

		printf("An error occurred while optimizing.\n");
		MSK_getcodedesc(r,
			symname,
			desc);
		printf("Error %s - '%s'\n", symname, desc);
	}
}
