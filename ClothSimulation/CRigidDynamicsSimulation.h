
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"
#include "Rigid.h"
#include "odeBoxBox.h"


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


	struct SBone
	{
		vec3f Position;
		vec3f Anchor;
		vec3f Extents;
		float Density;
		color3f Color = Colors::Red;
		vector<SBone *> Children;
		SBone * Parent = nullptr;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
		ion::Graphics::CUniform<color3f> ColorUniform;
		
		vector<vec3f> RotationFrames;
		vector<vec3f> VelocityFrames;

		glm::mat4 GetRotationMatrix();
	};

	struct SPlane
	{
		vec3d Normal;
		float Distance;
	};

protected:

	vector<SBone *> Bones;

	mutex SystemMutex;

	int VisibleFrame = 0;
	SBone * SelectedBone = nullptr;

};
