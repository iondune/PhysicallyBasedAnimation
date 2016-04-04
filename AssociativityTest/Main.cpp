
#include <ionEngine.h>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>


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

int main()
{
	LARGE_INTEGER frequency, start, stop;
	QueryPerformanceFrequency(&frequency);

	static int const Size = 100;
	Eigen::MatrixXd const A = RandomMatrix(Size);
	Eigen::MatrixXd const B = RandomMatrix(Size);
	Eigen::MatrixXd const C = RandomMatrix(Size);
	Eigen::MatrixXd const D = RandomMatrix(Size);
	Eigen::MatrixXd const E = RandomMatrix(Size);

	Eigen::VectorXd const x = RandomVector(Size);
	Eigen::VectorXd y;

	Eigen::VectorXd const a = A * x;
	Eigen::MatrixXd const aT = a.transpose();


	double avgA = 0, avgB = 0;
	for (int t = 0; t < Attempts; ++ t)
	{
		QueryPerformanceCounter(&start);
		for (int i = 0; i < Trials; ++ i)
		{
			y = aT * B.ldlt().solve(a);
			//y = (A * (B * (C * (D * (E * x)))));
		}
		QueryPerformanceCounter(&stop);
		double TimeA = Elapsed(start, stop, frequency);

		QueryPerformanceCounter(&start);
		for (int i = 0; i < Trials; ++ i)
		{
			y = aT * B.inverse() * a;
			//y = (((((A * B) * C) * D) * E) * x);
		}
		QueryPerformanceCounter(&stop);
		double TimeB = Elapsed(start, stop, frequency);

		printf("%.9f & %.9f\n", TimeA, TimeB);

		if (t)
		{
			avgA += TimeA / (Attempts - 1);
			avgB += TimeB / (Attempts - 1);
		}
	}
	printf("%.9f & %.9f\n", avgA, avgB);

	WaitForUser();
	return 0;
}
