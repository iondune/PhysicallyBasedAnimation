
#include "Util.h"


Eigen::Vector3d ToEigen(vec3d const & v)
{
	Eigen::Vector3d ret;
	ret << v.X, v.Y, v.Z;
	return ret;
}

Eigen::Vector3f ToEigen(vec3f const & v)
{
	Eigen::Vector3f ret;
	ret << v.X, v.Y, v.Z;
	return ret;
}

Eigen::Vector2d ToEigen(vec2d const & v)
{
	Eigen::Vector2d ret;
	ret << v.X, v.Y;
	return ret;
}

Eigen::Array<double, 1, 1> ToEigen(double const x)
{
	Eigen::Array<double, 1, 1> ret;
	ret << x;
	return ret;
}

vec3d ToIon3D(Eigen::Vector3d const & v)
{
	return vec3d(v.x(), v.y(), v.z());
}

vec3f ToIon3D(Eigen::Vector3f const & v)
{
	return vec3f(v.x(), v.y(), v.z());
}

vec2d ToIon2D(Eigen::Vector2d const & v)
{
	return vec2d(v.x(), v.y());
}
