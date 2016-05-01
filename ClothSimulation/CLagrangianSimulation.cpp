
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
	Player = new SParticle();
	Player->Mass = Settings.mass;
	Player->IsShip = true;
	Particles.push_back(Player);
}

vec2d SquareWithSign(vec2d const & Input)
{
	return vec2d(
		Sq(Input.X) * Sign(Input.X),
		Sq(Input.Y) * Sign(Input.Y)
	);
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

		double const Gravity = -0.98;
		double const GlobalEngineScale = 0.1;
		double const Friction = 0.06;

		Eigen::Matrix2d const A = J_Transpose * M * J;
		Eigen::Vector2d const b = J_Transpose * M * J * ToEigen(particle->Velocity) +
			TimeDelta * (
				J_Transpose * ToEigen(particle->Mass * vec3d(0, 0, Gravity) + particle->EngineForce * GlobalEngineScale) +
				ToEigen(vec2d(-(SquareWithSign(particle->Velocity)) * (Settings.Friction ? Friction : 0.f)))
			);

		Eigen::Vector2d const Result = A.ldlt().solve(b);

		particle->Velocity = ToIon2D(Result);
		particle->LastPosition = particle->Position;
		particle->Position += TimeDelta * particle->Velocity;
		particle->Position.X = fmod(particle->Position.X, 2 * Constants64::Pi);
		particle->Position.Y = fmod(particle->Position.Y, 2 * Constants64::Pi);


		vec3f Forward = vec3f(1, 0, 0)
			.RotateAround(vec3f(0, 1, 0), (float) particle->Heading)
			.RotateAround(vec3f(0, 0, 1), Constants32::Pi / 2 - (float) particle->Position.X)
			.RotateAround(vec3f(0, 1, 0), (float) -particle->Position.Y);

		std::swap(Forward.Y, Forward.Z);
		particle->ForwardVector = Forward;
		particle->EngineForce = vec3d(particle->ForwardVector) * particle->Thrust;

		if (! particle->IsShip)
		{
			particle->Thrust *= 0.95;
		}
	}
}

void CLagrangianSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	if (! BoundaryMesh)
	{
		BoundaryMesh = new CSimpleMeshSceneObject();
		BoundaryMesh->SetMesh(CGeometryCreator::CreateTorus((float) RingRadius, (float) TubeRadius, 64, 16));
		BoundaryMesh->SetShader(Application->DiffuseShader);
		BoundaryMesh->SetUniform("uColor", CUniform<color3f>(Colors::Cyan));
		BoundaryMesh->SetFeatureEnabled(EDrawFeature::Wireframe, true);
		Application->RenderPass->AddSceneObject(BoundaryMesh);
	}

	for (auto Particle : Particles)
	{
		if (! Particle->MeshObject)
		{
			Particle->MeshObject = new CSimpleMeshSceneObject();
			Particle->MeshObject->SetMesh(Particle->IsShip ? PlayerMesh : MissileMesh);
			Particle->MeshObject->SetScale(0.04f);
			Particle->MeshObject->SetShader(Application->MeshShader);
			Particle->MeshObject->SetRotationOrder(ERotationOrder::XYZ);
			Application->RenderPass->AddSceneObject(Particle->MeshObject);
		}

		if (! Particle->ExhaustObject)
		{
			Particle->ExhaustObject = new CParticleSystemSceneObject();
			Particle->ExhaustObject->Shader = Application->ParticleShader;
			Particle->ExhaustObject->Texture = Application->FireTexture1;
			Particle->ExhaustObject->SetScale(0.02f);
			if (! Particle->IsShip)
			{
				Particle->ExhaustObject->Settings.UpwardsMax *= 9.f;
				Particle->ExhaustObject->Settings.UpwardsMin *= 9.f;
			}
			Application->RenderPass->AddSceneObject(Particle->ExhaustObject);
		}
	}
}

void CLagrangianSimulation::UpdateSceneObjects()
{
	for (auto Particle : Particles)
	{
		//if (Particle->IsShip)
		{
			Particle->ExhaustObject->Settings.EmitCount = (int) (3.f + 5.f / 0.3f * Particle->Thrust);
			Particle->ExhaustObject->Settings.MinSize = (0.6f + 0.3f / 0.3f * Particle->Thrust) * 0.8f * 0.006f;
			Particle->ExhaustObject->Settings.MaxSize = (0.6f + 0.3f / 0.3f * Particle->Thrust) * 3.2f * 0.006f;
		}

		Particle->MeshObject->SetPosition(QToCartesian(Particle->Position));
		Particle->ExhaustObject->SetPosition(QToCartesian(Particle->Position));

		static glm::vec3 const X = glm::vec3(1, 0, 0);
		static glm::vec3 const Y = glm::vec3(0, 1, 0);
		static glm::vec3 const Z = glm::vec3(0, 0, 1);

		glm::mat4 Rotation = glm::mat4(1.f);
		Rotation = glm::rotate(Rotation, (float) -Particle->Position.Y, Y);
		Rotation = glm::rotate(Rotation, Constants32::Pi / 2 - (float) Particle->Position.X, Z);
		Rotation = glm::rotate(Rotation, Particle->Heading, Y);

		Particle->MeshObject->SetRotation(Rotation);
		Particle->ExhaustObject->SetRotation(Rotation);
	}
}

vec3f CLagrangianSimulation::QToCartesian(vec2f const & Angles)
{
	return vec3f(
		(float) ((RingRadius + TubeRadius * Cos(Angles.X)) * Cos(Angles.Y)),
		-(float) (TubeRadius * Sin(Angles.X)),
		(float) ((RingRadius + TubeRadius * Cos(Angles.X)) * Sin(Angles.Y))
	);
}

vec3f CLagrangianSimulation::ClosestCenter(vec2f const & Angles)
{
	return vec3f(
		(float) ((RingRadius) * Cos(Angles.Y)),
		0,
		(float) ((RingRadius) * Sin(Angles.Y))
	);
}
