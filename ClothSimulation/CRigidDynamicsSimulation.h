
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

	void PickObject(ray3f const & Ray);


	struct SBox
	{
		vec3d Extent;
		Eigen::Vector6d Mass;
		double m;
		color3f Color = Colors::Red;
		int Index = 0;
		bool Fixed = false;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
		ion::Graphics::CUniform<color3f> ColorUniform;
		
		Eigen::Matrix4d Position;
		Eigen::Vector6d ReactionForce = Eigen::Vector6d::Zero();
		Eigen::Vector3d AppliedForce = Eigen::Vector3d::Zero();
		Eigen::Vector3d AppliedTorque = Eigen::Vector3d::Zero();
		Eigen::Vector3d LocalForce = Eigen::Vector3d::Zero();
		vec3d w = 0;
		vec3d v = 0;

		Eigen::Vector6d GetPhi() const;
		vec3d GetTranslation();

		vec3d OriginalTranslation;
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
		int Index = 0;

		ion::Scene::CSimpleMeshSceneObject * SceneObject = nullptr;
		ion::Scene::CCoordinateFrameSceneObject * CoordianteFrame = nullptr;
	};

	SSettings Settings;

	vector<SBox *> Boxes;
	vector<SPlane> Planes;
	vector<SJoint *> Joints;

protected:

	int BodyMatrixSize = 0;
	int JointMatrixSize = 0;

	SBox * SelectedBox = nullptr;

	bool PlaneObjectsCreated = false;

};
