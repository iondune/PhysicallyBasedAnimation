
#pragma once

#include <Eigen/Dense>


namespace Eigen
{

	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix<double, 6, 6> Matrix6d;
	typedef Matrix<double, 3, 6> Matrix3x6d;
	typedef Matrix<double, 6, 3> Matrix6x3d;

}


class Rigid
{
private:
	Rigid();

public:
	static Eigen::Matrix4d inverse(const Eigen::Matrix4d &E);
	static Eigen::Matrix3x6d gamma(const Eigen::Vector3d &r);
	static Eigen::Matrix6d adjoint(const Eigen::Matrix4d &E);
	static Eigen::Matrix3d bracket3(const Eigen::Vector3d &a);
	static Eigen::Matrix4d bracket6(const Eigen::Vector6d &a);
	static Eigen::Vector3d unbracket3(const Eigen::Matrix3d &A);
	static Eigen::Vector6d unbracket6(const Eigen::Matrix4d &A);
	static Eigen::Matrix4d integrate(const Eigen::Matrix4d &E0, const Eigen::VectorXd &phi, double h);
};
