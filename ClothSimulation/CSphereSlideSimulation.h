
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"


class CSphereSlideSimulation : public ISimulation
{

public:

	void Setup();
	void SimulateStep(double const TimeDelta);

	void AddSceneObjects(ion::Scene::CRenderPass * RenderPass);
	void UpdateSceneObjects(uint const CurrentFrame);

	vec3d const & GetPosition() const;
	double GetRadius() const;

protected:

	vector<vec3d> PositionFrames;
	double Radius;
	double TotalTime = 0;

	mutex PositionMutex;

	ion::Scene::CSimpleMeshSceneObject * SphereObject = nullptr;

};
