
#include "CSimulationSystem.h"

using namespace ion;


void CSimulationSystem::Start(ion::Scene::CRenderPass * RenderPass)
{
	this->RenderPass = RenderPass;

	for (auto Simulation : Simulations)
	{
		Simulation->Setup();
		Simulation->AddSceneObjects(RenderPass);
		Simulation->UpdateSceneObjects(DisplayedFrame);
	}

	SimulationThread = thread([this]()
	{
		while (Running)
		{
			if (Simulating)
			{
				for (auto Simulation : Simulations)
				{
					Simulation->SimulateStep(TimeStep);
				}
				SimulatedFrames ++;
			}
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
	});
}

void CSimulationSystem::Stop()
{
	Running = false;
	SimulationThread.join();
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

	ImGui::End();
}

void CSimulationSystem::AddSimulation(ISimulation * Simulation)
{
	Simulations.push_back(Simulation);
}
