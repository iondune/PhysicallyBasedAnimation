
#include "CLagrangianSimulation.h"
#include "CApplication.h"
#include "Util.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


CLagrangianSimulation::CLagrangianSimulation()
{
	SParticle * Particle = new SParticle();
	Particle->Mass = Settings.mass;
	Particle->Position = (0.0);
	Particle->Velocity = (0.0);
	Particles.push_back(Particle);

	AddSceneObjects();
	UpdateSceneObjects();
}

void CLagrangianSimulation::SimulateStep(double const TimeDelta)
{
	for (SParticle * particle : Particles)
	{
		double const r = TubeRadius;
		double const R = RingRadius;
		double const theta = particle->Position.X;
		double const phi = particle->Position.Y;

		Eigen::Matrix3Xd J;
		J.resize(Eigen::NoChange, 2);
		J(0, 0) = -r * sin(theta) * cos(phi);
		J(1, 0) = -r * sin(theta) * sin(phi);
		J(2, 0) = -r * cos(theta);
		J(0, 1) = -(R + r * cos(theta)) * sin(phi);
		J(1, 1) = (R + r * cos(theta)) * cos(phi);
		J(2, 1) = 0;

		Eigen::Matrix3d const M = Eigen::Matrix3d::Identity() * particle->Mass;

		Eigen::Matrix2Xd J_Transpose;
		J_Transpose.resize(Eigen::NoChange, 3);
		J_Transpose = J.transpose();

		double const Gravity = 0.98;

		Eigen::Matrix2d const A = J_Transpose * M * J;
		Eigen::Vector2d const b = J_Transpose * M * J * ToEigen(particle->Velocity) +
			TimeDelta * J_Transpose * (particle->Mass * ToEigen(vec3d(0, 0, Gravity)));

		Eigen::Vector2d const Result = A.ldlt().solve(b);

		particle->Velocity = ToIon2D(Result);
		particle->Position += TimeDelta * particle->Velocity;
	}
}

void CLagrangianSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	BoundaryMesh = new CSimpleMeshSceneObject();
	BoundaryMesh->SetMesh(CGeometryCreator::CreateTorus((float) RingRadius, (float) TubeRadius, 20, 10));
	BoundaryMesh->SetShader(Application->DiffuseShader);
	BoundaryMesh->SetUniform("uColor", CUniform<color3f>(Colors::Cyan));
	BoundaryMesh->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	Application->RenderPass->AddSceneObject(BoundaryMesh);

	for (auto Particle : Particles)
	{
		Particle->DebugObject = new CSimpleMeshSceneObject();
		Particle->DebugObject->SetMesh(Application->SphereMesh);
		Particle->DebugObject->SetScale(0.02f);
		Particle->DebugObject->SetShader(Application->DiffuseShader);
		Particle->DebugObject->SetUniform("uColor", CUniform<color3f>(Colors::Red));
		Application->RenderPass->AddSceneObject(Particle->DebugObject);
	}
}

void CLagrangianSimulation::UpdateSceneObjects()
{
	for (auto Particle : Particles)
	{
		Particle->DebugObject->SetPosition(QToCartesian(Particle->Position));
	}
}

vec3f CLagrangianSimulation::QToCartesian(vec2f const & Angles)
{
	return vec3f(
		(float) ((RingRadius + TubeRadius * Cos(Angles.X)) * Cos(Angles.Y)),
		(float) (TubeRadius * Sin(Angles.X)),
		(float) ((RingRadius + TubeRadius * Cos(Angles.X)) * Sin(Angles.Y))
	);
}
