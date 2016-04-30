
#pragma once

#include <ionEngine.h>


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

		vec2d Position;
		vec2d Velocity;
	};

	struct SSettings
	{
		double mass = 0.1;
	};

protected:

	SSettings Settings;

	vector<SParticle *> Particles;

	double RingRadius = 2.0;
	double TubeRadius = 0.25;

	vec3f QToCartesian(vec2f const & Angles);

	ion::Scene::CSimpleMeshSceneObject * BoundaryMesh = nullptr;

};
