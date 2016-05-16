
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"
#include "Rigid.h"


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
		vec3d Extent;
		float Mass;
		color3f Color = Colors::Red;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
		
		vector<Eigen::Matrix4d> PositionFrames;
		vector<vec3d> wFrames;
		vector<vec3d> vFrames;
	};

	struct SPlane
	{
		vec3d Normal;
		float Distance;
	};

	struct SSettings
	{
		vec3d Center = vec3d(0, 0.25, 0);
		vec3d Size = vec3d(0.1, 0.05, 0.05);
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
