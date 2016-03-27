
#include "CSimulationSystem.h"


void CSimulationSystem::Start(ion::Scene::CRenderPass * RenderPass)
{
	this->RenderPass = RenderPass;

	ClothSimulation.Setup();
	ClothSimulation.AddSceneObjects(RenderPass);
	ClothSimulation.UpdateSceneObjects(DisplayedFrame);

	SimulationThread = thread([this]()
	{
		while (Running)
		{
			if (Simulating)
			{
				ClothSimulation.SimulateStep(TimeStep);
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

		if (StepAccumulator > TimeStep)
		{
			StepAccumulator = 0;

			if (DisplayedFrame < MaxFrames)
			{
				ClothSimulation.UpdateSceneObjects(++DisplayedFrame);
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
	ImGui::Text("Simulated Frames: %d", MaxFrames);
	if (ImGui::SliderInt("Current Frame", &DisplayedFrame, 0, MaxFrames))
	{
		ClothSimulation.UpdateSceneObjects(DisplayedFrame);
		Paused = true;
	}
	if (ImGui::Button("<< Previous"))
	{
		if (DisplayedFrame > 0)
		{
			ClothSimulation.UpdateSceneObjects(--DisplayedFrame);
		}
		Paused = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Next >>"))
	{
		if (DisplayedFrame < MaxFrames)
		{
			ClothSimulation.UpdateSceneObjects(++DisplayedFrame);
		}
		Paused = true;
	}
	ImGui::End();
}
