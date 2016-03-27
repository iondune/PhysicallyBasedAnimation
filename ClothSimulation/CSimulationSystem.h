
#pragma once

#include <ionEngine.h>
#include "CClothSimulation.h"


class CSimulationSystem
{

public:

	void Start(ion::Scene::CRenderPass * RenderPass);
	void Stop();

	void Update();
	void GUI();

protected:

	CClothSimulation ClothSimulation;

	thread SimulationThread;
	int DisplayedFrame = 0;
	int MaxFrames = 1;

	std::atomic<int> SimulatedFrames = 1;

	bool Running = true;
	bool Simulating = false;
	bool Paused = false;

	double const TimeStep = 1e-2;
	double StepAccumulator = 0;

	ion::Scene::CRenderPass * RenderPass = nullptr;

};
