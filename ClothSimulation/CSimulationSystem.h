
#pragma once

#include <ionEngine.h>


class ISimulation
{

public:

	virtual void Setup() = 0;
	virtual void SimulateStep(double const TimeDelta) = 0;
	virtual void GUI() = 0;
	virtual void Reset() = 0;

	virtual void UpdateSceneObjects(uint const CurrentFrame) = 0;

};

class CSimulationSystem : public Singleton<CSimulationSystem>
{

public:

	void Start(ion::Scene::CRenderPass * RenderPass);
	void Stop();

	void Update();
	void GUI();

	void Reset();

	void AddSimulation(ISimulation * Simulation);

	double TimeStep = 1e-3;
	int TimeStepPower = -3;

protected:

	vector<ISimulation *> Simulations;

	int DisplayedFrame = 0;
	int MaxFrames = 1;

	std::atomic<int> SimulatedFrames = 1;

	bool Running = true;
	bool Simulating = false;
	bool Paused = false;

	double StepAccumulator = 0;

	int PlaybackSpeed = 0;

	ion::Scene::CRenderPass * RenderPass = nullptr;

private:

	friend class Singleton<CSimulationSystem>;
	CSimulationSystem()
	{}

};
