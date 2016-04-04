
#include <ionEngine.h>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>


Eigen::MatrixXd RandomMatrix(int const Size)
{
	Eigen::MatrixXd ret;
	ret.resize(Size, Size);

	for (int i = 0; i < Size; ++ i)
	{
		for (int j = 0; j < Size; ++ j)
		{
			ret(i, j) = nrand();
		}
	}

	return ret;
}

Eigen::VectorXd RandomVector(int const Size)
{
	Eigen::VectorXd ret;
	ret.resize(Size);

	for (int i = 0; i < Size; ++ i)
	{
		ret(i) = nrand();
	}

	return ret;
}

int const Attempts = 11;
int const Trials = 10000;

double Elapsed(LARGE_INTEGER start, LARGE_INTEGER stop, LARGE_INTEGER frequency)
{
	return (double) (stop.QuadPart - start.QuadPart) / (double) (frequency.QuadPart);
}

struct ConjugateGradient
{
	uint Iterations = 1;
	uint Size = 4;

	Eigen::VectorXd x0, b;
	Eigen::MatrixXd A;

	Eigen::VectorXd Solve() const
	{
		Eigen::VectorXd r = b - A * x0;
		Eigen::VectorXd p = r;
		Eigen::VectorXd x = x0;

		for (uint k = 0; k < Iterations; ++ k)
		{
			//auto row = p.transpose() * A;
			//double bottom = row * p;
			//double top = r.transpose() * r;

			double const alpha =
				//top / bottom
				(double) (r.transpose() * r) /
				(double) (p.transpose() * A * p);
				;

			Eigen::VectorXd const x_next = x + alpha * p;
			Eigen::VectorXd const r_next = r - alpha * A * p;

			double const beta =
				(double) (r_next.transpose() * r_next) /
				(double) (r.transpose() * r);

			Eigen::VectorXd const p_next = r_next + beta * p;

			r = r_next;
			p = p_next;
			x = x_next;
		}

		return x;
	}
};

int main()
{
	LARGE_INTEGER frequency, start, stop;
	QueryPerformanceFrequency(&frequency);

	srand(123);

	static int const Size = 4;
	Eigen::MatrixXd const A = RandomMatrix(Size);
	Eigen::MatrixXd const SPD = A * A.transpose();

	Eigen::VectorXd const b = RandomVector(Size);
	Eigen::VectorXd x0;
	x0.resize(Size);
	x0.setZero();

	Eigen::ConjugateGradient<Eigen::MatrixXd> cg;
	cg.setMaxIterations(1);
	cg.setTolerance(1e-3);
	cg.compute(A);

	ConjugateGradient me_cg;
	me_cg.A = A;
	me_cg.b = b;
	me_cg.x0 = x0;

	Eigen::VectorXd x = cg.solveWithGuess(b, x0);
	cout << "x =" << endl;
	cout << x << endl;
	cout << endl;

	x = me_cg.Solve();
	cout << "x =" << endl;
	cout << x << endl;
	cout << endl;

	//double avgA = 0, avgB = 0;
	//for (int t = 0; t < Attempts; ++ t)
	//{
	//	QueryPerformanceCounter(&start);
	//	for (int i = 0; i < Trials; ++ i)
	//	{
	//	}
	//	QueryPerformanceCounter(&stop);
	//	double TimeA = Elapsed(start, stop, frequency);

	//	QueryPerformanceCounter(&start);
	//	for (int i = 0; i < Trials; ++ i)
	//	{
	//	}
	//	QueryPerformanceCounter(&stop);
	//	double TimeB = Elapsed(start, stop, frequency);

	//	printf("%.9f & %.9f\n", TimeA, TimeB);

	//	if (t)
	//	{
	//		avgA += TimeA / (Attempts - 1);
	//		avgB += TimeB / (Attempts - 1);
	//	}
	//}
	//printf("%.9f & %.9f\n", avgA, avgB);

	WaitForUser();
	return 0;
}
