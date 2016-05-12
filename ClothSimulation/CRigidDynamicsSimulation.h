
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"


class CRigidDynamicsSimulation : public ISimulation
{

public:

	CRigidDynamicsSimulation();

	void Setup();
	void SimulateStep(double const TimeDelta);
	void GUI();
	void Reset();

	void AddSceneObjects();
	void UpdateSceneObjects(uint const CurrentFrame);

	void PickParticle(ray3f const & Ray);


	struct SBox
	{
		vec3f Extent;
		float Mass;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
		
		vector<glm::mat4> PositionFrames;
		vector<vec3f> wFrames;
		vector<vec3f> vFrames;
	};

	struct SPlane
	{
		vec3f Normal;
		float Distance;
	};

	struct SSettings
	{
		vec3f Center = vec3f(0, 0.25f, 0);
		vec3f Size = vec3f(0.1f, 0.05f, 0.05f);
	};

protected:

	SSettings Settings;

	vector<SBox *> Boxes;
	vector<SPlane> Planes;

	mutex SystemMutex;

	SBox * GetParticle(vec2i const & Index);

	int VisibleFrame = 0;
	SBox * SelectedParticle = nullptr;

	bool PlaneObjectsCreated = false;

};
