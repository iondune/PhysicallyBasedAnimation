
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

		ion::Scene::CSimpleMeshSceneObject * DebugObject = nullptr;
		CParticleSystemSceneObject * ExhaustObject = nullptr;

		vec2d Position;
		vec2d LastPosition;
		vec2d Velocity;
		vec3d EngineForce;

		float Heading = 0;
	};

	struct SSettings
	{
		double mass = 0.1;
	};

	SParticle * Player = nullptr;

	vec3f QToCartesian(vec2f const & Angles);
	vec3f ClosestCenter(vec2f const & Angles);

	ion::Scene::CSimpleMesh * PlayerMesh = nullptr;

protected:

	SSettings Settings;

	vector<SParticle *> Particles;

	double RingRadius = 2.0;
	double TubeRadius = 0.25;

	ion::Scene::CSimpleMeshSceneObject * BoundaryMesh = nullptr;

};
