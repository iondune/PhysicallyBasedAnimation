
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

	struct SParticle
	{
		double Radius;
		double Mass;
		int Index;
		bool IsFixed;

		bool IsConstrained = false;
		
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

	struct SSettings
	{
		int rows = 10;
		int cols = 10;
		double mass = 0.1;
		double stiffness = 1e3;
		vec2d damping = vec2d(0.0, 1.0);
	};

protected:

	SSettings Settings;

	int Rows = 0;
	int Columns = 0;
	int MatrixSize = 0;
	vec2d Damping;

	vector<SParticle *> Particles;
	vector<SSpring *> Springs;

	mutex ParticlesMutex;

	SParticle * GetParticle(vec2i const & Index);

	ion::Scene::CSimpleMesh * ClothMesh = nullptr;
	ion::Scene::CSimpleMeshSceneObject * ClothObjectFront = nullptr;
	ion::Scene::CSimpleMeshSceneObject * ClothObjectBack = nullptr;

};
