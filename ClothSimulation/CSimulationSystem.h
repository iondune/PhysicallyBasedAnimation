
#pragma once

#include <ionEngine.h>


class ISimulation
{

public:

	virtual void Setup() = 0;
	virtual void SimulateStep(double const TimeDelta) = 0;

	virtual void AddSceneObjects(ion::Scene::CRenderPass * RenderPass) = 0;
	virtual void UpdateSceneObjects(uint const CurrentFrame) = 0;

};

class CSimulationSystem
{

public:

	void Start(ion::Scene::CRenderPass * RenderPass);
	void Stop();

	void Update();
	void GUI();

	void AddSimulation(ISimulation * Simulation);

protected:

	vector<ISimulation *> Simulations;

	thread SimulationThread;
	int DisplayedFrame = 0;
	int MaxFrames = 1;

	std::atomic<int> SimulatedFrames = 1;

	bool Running = true;
	bool Simulating = false;
	bool Paused = false;

	double const TimeStep = 1e-3;
	double StepAccumulator = 0;

	int PlaybackSpeed = 0;

	ion::Scene::CRenderPass * RenderPass = nullptr;

};
