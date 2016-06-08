
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


	struct SBox
	{
		vec3d Extent;
		Eigen::Vector6d Mass;
		double m;
		color3f Color = Colors::Red;
		int Index = 0;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
		ion::Graphics::CUniform<color3f> ColorUniform;
		
		vector<Eigen::Matrix4d> PositionFrames;
		vector<Eigen::Vector6d> ReactionForceFrames;
		vector<vec3d> wFrames;
		vector<vec3d> vFrames;
		vector<vector<Contacts>> contactFrames;

		Eigen::Vector6d GetPhi() const;
	};

	struct SPlane
	{
		vec3d Normal;
		float Distance;
	};

	struct SSettings
	{
		vec3d Center = vec3d(0, 1.25, 0);
		vec3d Size = vec3d(0.1, 0.05, 0.05);
	};

	struct SJoint
	{
		SBox * Body_i = nullptr;
		SBox * Body_k = nullptr;

		Eigen::Matrix4d JointFrame;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
	};

protected:

	SSettings Settings;

	vector<SBox *> Boxes;
	vector<SPlane> Planes;
	vector<SJoint *> Joints;

	mutex SystemMutex;

	SBox * GetParticle(vec2i const & Index);

	int VisibleFrame = 0;
	SBox * SelectedParticle = nullptr;

	bool PlaneObjectsCreated = false;

};
