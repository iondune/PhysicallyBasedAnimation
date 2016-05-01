
#pragma once

#include <ionEngine.h>
#include "CParticleSystemSceneObject.h"


class CLagrangianSimulation
{

public:

	CLagrangianSimulation();

	void SimulateStep(double const TimeDelta);

	void AddSceneObjects();
	void UpdateSceneObjects();

	struct SParticle
	{
		double Mass;

		ion::Scene::CSimpleMeshSceneObject * MeshObject = nullptr;
		CParticleSystemSceneObject * ExhaustObject = nullptr;

		vec2d Position;
		vec2d LastPosition;
		vec2d Velocity;
		vec3d EngineForce;

		double Thrust = 0;
		float Heading = 0;
		vec3f ForwardVector;

		bool IsShip = false;
	};

	struct SSettings
	{
		double mass = 0.1;
		bool Friction = false;
	};

	SParticle * Player = nullptr;

	vec3f QToCartesian(vec2f const & Angles);
	vec3f ClosestCenter(vec2f const & Angles);

	ion::Scene::CSimpleMesh * PlayerMesh = nullptr;
	ion::Scene::CSimpleMesh * MissileMesh = nullptr;

	vector<SParticle *> Particles;

	SSettings Settings;

protected:

	double const RingRadius = 3.5;
	double const TubeRadius = 0.75;

	ion::Scene::CSimpleMeshSceneObject * BoundaryMesh = nullptr;

};
