
#include "CSimulationSystem.h"
#include "CRigidDynamicsSimulation.h"

using namespace ion;


void CSimulationSystem::Start(ion::Scene::CRenderPass * RenderPass)
{
	this->RenderPass = RenderPass;

	for (auto Simulation : Simulations)
	{
		Simulation->Setup();
	}
}

void CSimulationSystem::Stop()
{
	Running = false;
}

void CSimulationSystem::Update()
{
	SingletonPointer<CTimeManager> TimeManager;

	if (! Paused)
	{
		StepAccumulator += TimeManager->GetElapsedTime();

		double TimeNeeded = TimeStep / pow(10, PlaybackSpeed);

		if (StepAccumulator > TimeNeeded)
		{
			StepAccumulator = 0;

			SimulationMutex.lock();
			for (auto Simulation : Simulations)
			{
				Simulation->SimulateStep(TimeStep);
			}
			SimulatedFrames ++;
			SimulationMutex.unlock();

			if (DisplayedFrame < MaxFrames)
			{
				++DisplayedFrame;
				for (auto Simulation : Simulations)
				{
					Simulation->UpdateSceneObjects(DisplayedFrame);
				}
			}
		}
	}
}

void CSimulationSystem::GUI()
{
	MaxFrames = SimulatedFrames - 1;

	ImGui::SetNextWindowPos(ImVec2(10, 10));
	ImGui::Begin("Simulation");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::Separator();
	if (Simulating)
	{
		if (ImGui::Button("Stop"))
		{
			Simulating = false;
		}
	}
	else
	{
		if (ImGui::Button("Simulate"))
		{
			Simulating = true;
		}
	}
	ImGui::SameLine();
	if (Paused)
	{
		if (ImGui::Button("Play"))
		{
			Paused = false;
		}
	}
	else
	{
		if (ImGui::Button("Pause"))
		{
			Paused = true;
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset"))
	{
		Reset();
		Simulating = false;
		Paused = true;
	}

	if (ImGui::Button("Settings"))
	{
		ImGui::OpenPopup("Cloth Settings");
		Simulating = false;
		Paused = true;
	}

	ImGui::Text("Playback Speed: 1e%d", PlaybackSpeed);
	ImGui::SliderInt("Playback Speed", &PlaybackSpeed, -3, 0);

	bool UpdatedNeeded = false;

	ImGui::Text("Simulated Frames: %d", MaxFrames);
	if (ImGui::SliderInt("Current Frame", &DisplayedFrame, 0, MaxFrames))
	{
		UpdatedNeeded = true;
		Paused = true;
	}
	if (ImGui::Button("<< Previous"))
	{
		if (DisplayedFrame > 0)
		{
			--DisplayedFrame;
			UpdatedNeeded = true;
		}
		Paused = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Next >>"))
	{
		if (DisplayedFrame < MaxFrames)
		{
			++DisplayedFrame;
			UpdatedNeeded = true;
		}
		Paused = true;
	}

	if (UpdatedNeeded)
	{
		for (auto Simulation : Simulations)
		{
			Simulation->UpdateSceneObjects(DisplayedFrame);
		}
	}

	for (auto Simulation : Simulations)
	{
		Simulation->GUI();
	}

	ImGui::End();
}

void CSimulationSystem::Reset()
{
	Simulating = false;
	SimulationMutex.lock();

	SimulatedFrames = 1;
	DisplayedFrame = 0;

	for (auto Simulation : Simulations)
	{
		Simulation->Reset();
		Simulation->UpdateSceneObjects(DisplayedFrame);
	}

	SimulationMutex.unlock();
}

void CSimulationSystem::AddSimulation(ISimulation * Simulation)
{
	Simulations.push_back(Simulation);
}
