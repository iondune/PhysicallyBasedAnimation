
#pragma once

#include <ionMath.h>
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Sparse>


Eigen::Vector3d ToEigen(vec3d const & v);
Eigen::Vector3f ToEigen(vec3f const & v);
Eigen::Vector2d ToEigen(vec2d const & v);
Eigen::Array<double, 1, 1> ToEigen(double const x);

vec3d ToIon3D(Eigen::Vector3d const & v);
vec3f ToIon3D(Eigen::Vector3f const & v);
vec2d ToIon2D(Eigen::Vector2d const & v);
