
#include "CSimulationSystem.h"
#include "CRigidDynamicsSimulation.h"
#include "CApplication.h"

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

		while (StepAccumulator > TimeNeeded)
		{
			StepAccumulator -= TimeNeeded;

			for (auto Simulation : Simulations)
			{
				Simulation->SimulateStep(TimeStep);
			}

			SimulatedFrames ++;
		}

		for (auto Simulation : Simulations)
		{
			Simulation->FinishSteps();
			Simulation->UpdateSceneObjects();
		}
	}
}

void CSimulationSystem::GUI()
{
	SingletonPointer<CApplication> Application;

	ImGui::SetNextWindowPos(ImVec2(10, 10));
	ImGui::Begin("Simulation");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::Separator();
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

	ImGui::Text("Simulated Frames: %d", SimulatedFrames);
	ImGui::Separator();

	ImGui::Text("End Effector: %.3f %.3f %.3f", Application->GoalPosition.X, Application->GoalPosition.Y, Application->GoalPosition.Z);

	for (auto Simulation : Simulations)
	{
		Simulation->GUI();
	}

	ImGui::End();
}

void CSimulationSystem::Reset()
{
	Simulating = false;

	SimulatedFrames = 1;

	for (auto Simulation : Simulations)
	{
		Simulation->Reset();
		Simulation->UpdateSceneObjects();
	}
}

void CSimulationSystem::AddSimulation(ISimulation * Simulation)
{
	Simulations.push_back(Simulation);
}
