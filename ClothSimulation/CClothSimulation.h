
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"


class CClothSimulation : public ISimulation
{

public:

	CClothSimulation();

	void Setup();
	void SimulateStep(double const TimeDelta);
	void GUI();
	void Reset();

	void AddSceneObjects();
	void UpdateSceneObjects(uint const CurrentFrame);

	void PickParticle(ray3f const & Ray);

	enum class EConstraintType
	{
		None = 0,
		XAxis = 1,
		YAxis = 2,
		DownDiagonal = 3,
		UpDiagonal = 4
	};

	struct SParticle
	{
		double Radius;
		double Mass;
		int Index;
		bool IsFixed;

		EConstraintType ConstraintType = EConstraintType::None;

		ion::Scene::CSimpleMeshSceneObject * DebugObject = nullptr;
		
		vector<vec2d> PositionFrames;
		vector<vec2d> VelocityFrames;
	};

	struct SSpring
	{
		double Stiffness;
		double RestLength;

		SParticle * Particle0;
		SParticle * Particle1;

		SSpring(SParticle * Particle0, SParticle * Particle1, double const Stiffness)
		{
			this->Particle0 = Particle0;
			this->Particle1 = Particle1;
			this->Stiffness = Stiffness;
			this->RestLength = Particle0->PositionFrames[0].GetDistanceFrom(Particle1->PositionFrames[0]);
		}
	};

	struct SPlane
	{
		vec2d Normal;
		double Distance;
	};

	struct SSettings
	{
		int rows = 2;
		int cols = 2;
		double mass = 0.1;
		double stiffness = 1e2;
		vec2d damping = vec2d(0.0, 1.0);

		vec2d Center = vec2d(0, 0.25);
		vec2d Size = vec2d(0.5, 0.5);
	};

protected:

	SSettings Settings;

	int Rows = 0;
	int Columns = 0;
	int MatrixSize = 0;
	vec2d Damping;

	vector<SParticle *> Particles;
	vector<SSpring *> Springs;
	vector<SPlane> Planes;

	mutex ParticlesMutex;

	SParticle * GetParticle(vec2i const & Index);

	int VisibleFrame = 0;
	SParticle * SelectedParticle = nullptr;

	ion::Scene::CSimpleMesh * ClothMesh = nullptr;
	ion::Scene::CSimpleMeshSceneObject * ClothObjectFront = nullptr;
	ion::Scene::CSimpleMeshSceneObject * ClothObjectBack = nullptr;

	bool PlaneObjectsCreated = false;

};
