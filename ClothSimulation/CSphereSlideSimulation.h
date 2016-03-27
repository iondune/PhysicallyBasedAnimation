
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

protected:

	vector<vec3d> PositionFrames;
	double Radius;
	double TotalTime = 0;

	ion::Scene::CSimpleMeshSceneObject * SphereObject = nullptr;

};
