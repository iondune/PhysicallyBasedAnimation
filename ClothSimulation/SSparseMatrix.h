
#pragma once

#include <ionMath.h>
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Sparse>


struct SSparseMatrix
{
	map<vec2i, double> Elements;
	int const Size;

	SSparseMatrix(int const size)
		: Size(size)
	{}

	void Set(int const x, int const y, double const Value)
	{
		assert(x >= 0);
		assert(y >= 0);
		assert(x < Size);
		assert(y < Size);

		Elements[vec2i(x, y)] = Value;
	}

	double Get(int const x, int const y) const
	{
		assert(x >= 0);
		assert(y >= 0);
		assert(x < Size);
		assert(y < Size);

		double Value = 0;
		TryMapAccess(Elements, vec2i(x, y), Value);
		return Value;
	}

	void Add(int const x0, int const y0, Eigen::Matrix2d const & Mat)
	{
		assert(x0 >= 0);
		assert(y0 >= 0);
		assert(x0 + 1 < Size);
		assert(y0 + 1 < Size);

		for (int y = 0; y < 2; ++ y)
		{
			for (int x = 0; x < 2; ++ x)
			{
				Elements[vec2i(x + x0, y + y0)] += Mat(x, y);
			}
		}
	}

	void Subtract(int const x0, int const y0, Eigen::Matrix2d const & Mat)
	{
		assert(x0 >= 0);
		assert(y0 >= 0);
		assert(x0 + 1 < Size);
		assert(y0 + 1 < Size);

		for (int y = 0; y < 2; ++ y)
		{
			for (int x = 0; x < 2; ++ x)
			{
				Elements[vec2i(x + x0, y + y0)] -= Mat(x, y);
			}
		}
	}

	SSparseMatrix operator + (SSparseMatrix const & other) const
	{
		assert(other.Size == Size);

		SSparseMatrix ret = *this;

		for (auto & Element : other.Elements)
		{
			ret.Elements[Element.first] += Element.second;
		}

		return ret;
	}

	SSparseMatrix operator * (double const rhs) const
	{
		SSparseMatrix ret = *this;

		for (auto & Element : Elements)
		{
			ret.Elements[Element.first] *= rhs;
		}

		return ret;
	}

	Eigen::VectorXd operator * (Eigen::VectorXd const & rhs) const
	{
		assert(rhs.cols() == 1);
		assert(rhs.rows() == Size);
		Eigen::VectorXd ret;
		ret.resize(Size);

		for (int i = 0; i < Size; ++ i)
		{
			double ProductSum = 0;
			for (int j = 0; j < Size; ++ j)
			{
				ProductSum += Get(i, j) * rhs(j);
			}
			ret(i) = ProductSum;
		}

		return ret;
	}

	friend SSparseMatrix operator * (double const lhs, SSparseMatrix const & rhs)
	{
		SSparseMatrix ret = rhs;

		for (auto & Element : ret.Elements)
		{
			Element.second *= lhs;
		}

		return ret;
	}

	Eigen::SparseMatrix<double> ToEigen() const
	{
		std::vector<Eigen::Triplet<double>> Tuples;

		for (auto const & Element : Elements)
		{
			Tuples.push_back(Eigen::Triplet<double>(Element.first.X, Element.first.Y, Element.second));

			if (Element.first.X + 1 > Size || Element.first.Y + 1 > Size)
			{
				Log::Warn("Out-of-bounds element in sparse matrix.");
			}
		}

		Eigen::SparseMatrix<double> A(Size, Size);
		A.setFromTriplets(Tuples.begin(), Tuples.end());
		return A;
	}

	bool IsSymmetric() const
	{
		for (int i = 0; i < Size; ++ i)
		{
			double ProductSum = 0;
			for (int j = 0; j < i; ++ j)
			{
				if (Get(i, j) != Get(j, i))
					return false;
			}
		}

		return true;
	}

	uint CountNonZeroSymmetric() const
	{
		uint NonZero = 0;

		assert(IsSymmetric());

		for (int i = 0; i < Size; ++ i)
		{
			double ProductSum = 0;
			for (int j = 0; j <= i; ++ j)
			{
				if (Get(i, j) != 0)
					NonZero ++;
			}
		}

		return NonZero;
	}

	friend std::ostream & operator << (std::ostream & stream, SSparseMatrix const & rhs)
	{
		for (int i = 0; i < rhs.Size; ++ i)
		{
			for (int j = 0; j < rhs.Size; ++ j)
			{
				if (0 != j)
				{
					stream << " ";
				}
				stream << rhs.Get(i, j);
			}
			stream << endl;
		}

		return stream;
	}
};
