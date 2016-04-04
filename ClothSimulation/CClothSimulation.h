
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"
#include "CSphereSlideSimulation.h"


class CClothSimulation : public ISimulation
{

public:

	CClothSimulation();

	void Setup();
	void SimulateStep(double const TimeDelta);

	void AddSceneObjects(ion::Scene::CRenderPass * RenderPass);
	void UpdateSceneObjects(uint const CurrentFrame);

	struct SParticle
	{
		double Radius;
		double Mass;
		int Index;
		bool IsFixed;
		
		vector<vec3d> PositionFrames;
		vector<vec3d> VelocityFrames;
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

protected:

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
