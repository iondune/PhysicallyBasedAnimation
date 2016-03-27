
#include "CClothSimulation.h"
#include "CApplication.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


CClothSimulation::CClothSimulation(CSphereSlideSimulation * Sphere)
{
	this->Sphere = Sphere;
}

void CClothSimulation::Setup()
{
	int rows = 10;
	int cols = 10;
	double mass = 0.1;
	double stiffness = 1e2;
	vec2d damping(0.0, 1.0);
	vec3d x00(-0.25, 0.5, 0.0);
	vec3d x01(0.25, 0.5, 0.0);
	vec3d x10(-0.25, 0.5, -0.5);
	vec3d x11(0.25, 0.5, -0.5);

	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);

	Rows = rows;
	Columns = cols;
	Damping = damping;

	// Create particles
	double r = 0.02; // Used for collisions
	int nVerts = rows*cols;
	for (int i = 0; i < rows; ++i)
	{
		double u = i / (rows - 1.0);
		vec3d x0 = (1 - u)*x00 + u*x10;
		vec3d x1 = (1 - u)*x01 + u*x11;
		for (int j = 0; j < cols; ++j)
		{
			SParticle * p = new SParticle();
			Particles.push_back(p);

			p->Radius = r;
			double v = j / (cols - 1.0);
			p->PositionFrames.push_back((1 - v)*x0 + v*x1);
			p->VelocityFrames.push_back(0.0);
			p->Mass = mass / (nVerts);

			if (i == 0 && (j == 0 || j == cols - 1))
			{
				p->IsFixed = true;
				p->Index = -1;
			}
			else
			{
				p->IsFixed = false;
				p->Index = MatrixSize;
				MatrixSize += 3;
			}
		}
	}

	// Create x springs
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols - 1; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + 1;
			Springs.push_back(new SSpring(Particles[k0], Particles[k1], stiffness));
		}
	}

	// Create y springs
	for (int j = 0; j < cols; ++j) {
		for (int i = 0; i < rows - 1; ++i) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			Springs.push_back(new SSpring(Particles[k0], Particles[k1], stiffness));
		}
	}

	// Create shear springs
	for (int i = 0; i < rows - 1; ++i) {
		for (int j = 0; j < cols - 1; ++j) {
			int k00 = i*cols + j;
			int k10 = k00 + 1;
			int k01 = k00 + cols;
			int k11 = k01 + 1;
			Springs.push_back(new SSpring(Particles[k00], Particles[k11], stiffness));
			Springs.push_back(new SSpring(Particles[k10], Particles[k01], stiffness));
		}
	}
}

Eigen::Vector3d ToEigen(vec3d const & v)
{
	Eigen::Vector3d ret;
	ret << v.X, v.Y, v.Z;
	return ret;
}

Eigen::Array<double, 1, 1> ToEigen(double const x)
{
	Eigen::Array<double, 1, 1> ret;
	ret << x;
	return ret;
}

vec3d ToIon(Eigen::Vector3d const & v)
{
	return vec3d(v.x(), v.y(), v.z());
}

void CClothSimulation::SimulateStep(double const TimeDelta)
{
	static vec3d const Gravity = vec3d(0, -9.8, 0);

	Eigen::VectorXd v;
	Eigen::VectorXd f;
	Eigen::MatrixXd M;
	Eigen::MatrixXd K;

	M.resize(MatrixSize, MatrixSize);
	K.resize(MatrixSize, MatrixSize);
	v.resize(MatrixSize);
	f.resize(MatrixSize);

	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

	ParticlesMutex.lock();
	for (SParticle * particle : Particles)
	{
		if (! particle->IsFixed)
		{
			f.segment(particle->Index, 3) = ToEigen(Gravity * particle->Mass);
			v.segment(particle->Index, 3) = ToEigen(particle->VelocityFrames.back());
			M.block<1, 1>(particle->Index, particle->Index) = ToEigen(particle->Mass);
			M.block<1, 1>(particle->Index + 1, particle->Index + 1) = ToEigen(particle->Mass);
			M.block<1, 1>(particle->Index + 2, particle->Index + 2) = ToEigen(particle->Mass);
		}
	}

	for (SSpring * spring : Springs)
	{
		vec3d const PositionDelta = spring->Particle1->PositionFrames.back() - spring->Particle0->PositionFrames.back();
		double const CurrentLength = PositionDelta.Length();
		vec3d const SpringForce = spring->Stiffness * (CurrentLength - spring->RestLength) * PositionDelta / CurrentLength;

		Eigen::Matrix3d const I = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d const StiffnessMatrix = (spring->Stiffness / Sq(CurrentLength)) * (
			(1 - (CurrentLength - spring->RestLength) / CurrentLength) * (ToEigen(PositionDelta) * ToEigen(PositionDelta).transpose()) +
			((CurrentLength - spring->RestLength) / CurrentLength) * (Dot(PositionDelta, PositionDelta) * I)
			);

		if (! spring->Particle0->IsFixed)
		{
			f.segment(spring->Particle0->Index, 3) += ToEigen(SpringForce);
			K.block<3, 3>(spring->Particle0->Index, spring->Particle0->Index) += StiffnessMatrix;
		}
		if (! spring->Particle1->IsFixed)
		{
			f.segment(spring->Particle1->Index, 3) -= ToEigen(SpringForce);
			K.block<3, 3>(spring->Particle1->Index, spring->Particle1->Index) += StiffnessMatrix;
		}
		if (! spring->Particle0->IsFixed && ! spring->Particle1->IsFixed)
		{
			K.block<3, 3>(spring->Particle0->Index, spring->Particle1->Index) -= StiffnessMatrix;
			K.block<3, 3>(spring->Particle1->Index, spring->Particle0->Index) -= StiffnessMatrix;
		}
	}
	ParticlesMutex.unlock();

	Eigen::MatrixXd const D = Damping.X * TimeDelta * M + Damping.Y * Sq(TimeDelta) * K;

	Eigen::MatrixXd const A = M + D;
	Eigen::VectorXd const b = M * v + TimeDelta * f;

	Eigen::VectorXd const Result = A.ldlt().solve(b);

	ParticlesMutex.lock();
	for (SParticle * particle : Particles)
	{
		if (! particle->IsFixed)
		{
			particle->VelocityFrames.push_back(ToIon(Result.segment(particle->Index, 3)));
			particle->PositionFrames.push_back(particle->PositionFrames.back() + TimeDelta * particle->VelocityFrames.back());
		}
		else
		{
			particle->VelocityFrames.push_back(0);
			particle->PositionFrames.push_back(particle->PositionFrames.back());
		}
	}

	for (auto particle : Particles)
	{
		if (! particle->IsFixed)
		{
			vec3d Offset = particle->PositionFrames.back() - Sphere->GetPosition();
			double const Distance = Offset.Length();
			double const Radius = Sphere->GetRadius();

			if (Distance <= Radius)
			{
				vec3d const Normal = Offset.Normalize();
				Offset *= Radius;

				particle->PositionFrames.back() = Sphere->GetPosition() + Offset;
				particle->VelocityFrames.back() -= Dot(particle->VelocityFrames.back(), Normal) * Normal;
			}
		}
	}

	ParticlesMutex.unlock();
}

void CClothSimulation::AddSceneObjects(ion::Scene::CRenderPass * RenderPass)
{
	SingletonPointer<CApplication> Application;

	ClothMesh = new CSimpleMesh();

	for (int x = 0; x < Rows - 1; ++ x)
	{
		for (int y = 0; y < Columns - 1; ++ y)
		{
			uint const Start = (uint) ClothMesh->Vertices.size();

			CSimpleMesh::SVertex Vertex;
			for (int i = 0; i < 4; ++ i)
			{
				ClothMesh->Vertices.push_back(Vertex);
			}

			CSimpleMesh::STriangle Triangle;
			Triangle.Indices[0] = Start + 0;
			Triangle.Indices[1] = Start + 2;
			Triangle.Indices[2] = Start + 1;
			ClothMesh->Triangles.push_back(Triangle);

			Triangle.Indices[0] = Start + 0;
			Triangle.Indices[1] = Start + 3;
			Triangle.Indices[2] = Start + 2;
			ClothMesh->Triangles.push_back(Triangle);
		}
	}

	ClothObjectFront = new CSimpleMeshSceneObject();
	ClothObjectFront->SetMesh(ClothMesh);
	ClothObjectFront->SetShader(Application->ClothShader);
	ClothObjectFront->SetUniform("uColor", CUniform<color3f>(Colors::Red));
	ClothObjectFront->SetUniform("uFlipNormals", CUniform<int>(0));
	ClothObjectFront->SetFeatureEnabled(EDrawFeature::CullBack, true);
	RenderPass->AddSceneObject(ClothObjectFront);

	ClothObjectBack = new CSimpleMeshSceneObject();
	ClothObjectBack->SetMesh(ClothMesh);
	ClothObjectBack->SetShader(Application->ClothShader);
	ClothObjectBack->SetUniform("uColor", CUniform<color3f>(Colors::Yellow));
	ClothObjectBack->SetUniform("uFlipNormals", CUniform<int>(1));
	ClothObjectBack->SetFeatureEnabled(EDrawFeature::CullFront, true);
	RenderPass->AddSceneObject(ClothObjectBack);
}

void CClothSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	ParticlesMutex.lock();
	for (int x = 0; x < Rows - 1; ++ x)
	{
		for (int y = 0; y < Columns - 1; ++ y)
		{
			size_t const Start = (y + x * (Columns - 1)) * 4;

			for (size_t i = 0; i < 4; ++ i)
			{
				static vec2i const Offsets[] =
				{
					vec2i(0, 0),
					vec2i(0, 1),
					vec2i(1, 1),
					vec2i(1, 0),
				};

				ClothMesh->Vertices[Start + i].Position = GetParticle(vec2i(x, y) + Offsets[i])->PositionFrames[CurrentFrame];
				ClothMesh->Vertices[Start + i].Normal = vec3f(0, 0, 1);
			}
		}
	}
	ParticlesMutex.unlock();

	ClothObjectFront->SetMesh(ClothMesh);
	ClothObjectBack->SetMesh(ClothMesh);
}

CClothSimulation::SParticle * CClothSimulation::GetParticle(vec2i const & Index)
{
	return Particles[Index.X * Columns + Index.Y];
}
