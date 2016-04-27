
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"


class CLagrangianSimulation : public ISimulation
{

public:

	CLagrangianSimulation();

	void Setup();
	void SimulateStep(double const TimeDelta);
	void GUI();
	void Reset();

	void AddSceneObjects();
	void UpdateSceneObjects(uint const CurrentFrame);

	void PickParticle(ray3f const & Ray);

	struct SParticle
	{
		double Mass;

		ion::Scene::CSimpleMeshSceneObject * DebugObject = nullptr;

		vector<vec2d> PositionFrames;
		vector<vec2d> VelocityFrames;
	};

	struct SSettings
	{
		double mass = 0.1;
	};

protected:

	SSettings Settings;

	vector<SParticle *> Particles;

	mutex ParticlesMutex;

	int VisibleFrame = 0;
	SParticle * SelectedParticle = nullptr;

	double RingRadius = 2.0;
	double TubeRadius = 0.25;

	vec3f QToCartesian(vec2f const & Angles);

	ion::Scene::CSimpleMeshSceneObject * BoundaryMesh = nullptr;

};
