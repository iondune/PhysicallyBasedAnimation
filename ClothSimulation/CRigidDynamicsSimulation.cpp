
#include "CRigidDynamicsSimulation.h"
#include "CApplication.h"
#include "Util.h"


using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;



CRigidDynamicsSimulation::CRigidDynamicsSimulation()
{}

void CRigidDynamicsSimulation::Setup()
{
	SingletonPointer<CApplication> Application;

	SelectedBone = nullptr;
	for (auto Particle : Bones)
	{
		if (Particle->SceneObject)
		{
			Application->RenderPass->RemoveSceneObject(Particle->SceneObject);
		}
		delete Particle;
	}
	Bones.clear();

	SBone * Bone = new SBone();
	Bone->Color = Colors::Red;
	Bone->Position = vec3f(0, 1, 0);
	Bone->Anchor = vec3f(-0.3f, 0, 0);
	Bone->Extents = vec3f(0.3f, 0.1f, 0.1f);
	Bone->Density = 1.f;

	Bone->RotationFrames.push_back(vec3f());
	Bone->VelocityFrames.push_back(vec3f());

	Bones.push_back(Bone);

	AddSceneObjects();
	UpdateSceneObjects(0);
}

void CRigidDynamicsSimulation::SimulateStep(double const TimeDelta)
{
	static vec3d const Gravity = vec3d(0, -9.8, 0);


	//SystemMutex.lock();
	for (SBone * const Bone : Bones)
	{
		Bone->RotationFrames.push_back(Bone->RotationFrames.back() + vec3f(0, 0, TimeDelta));
		Bone->VelocityFrames.push_back(0.f);
	}
	//SystemMutex.unlock();
}

void CRigidDynamicsSimulation::GUI()
{
	SingletonPointer<CSimulationSystem> SimulationSystem;

	if (ImGui::BeginPopupModal("Cloth Settings"))
	{
		ImGui::SetWindowSize(ImVec2(500, 350), ImGuiSetCond_Once);

		ImGui::Text("Time Step: 1e%d", SimulationSystem->TimeStepPower);
		if (ImGui::SliderInt("Time Step Exponent", &SimulationSystem->TimeStepPower, -4, -1))
		{
			SimulationSystem->TimeStep = pow(10, SimulationSystem->TimeStepPower);
			SimulationSystem->Reset();
			Setup();
		}

		vec2d damping = vec2d(0.0, 1.0);

		if (ImGui::Button("Apply"))
		{
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

	if (SelectedBone)
	{
		if (ImGui::Begin("Rigid Body Info"))
		{
			ImGui::SetWindowSize(ImVec2(425, 250), ImGuiSetCond_Once);
			ImGui::SetWindowPos(ImVec2(1000, 350), ImGuiSetCond_Once);

			//vec4f position(0, 0, 0, 1);
			//vec4f scale(1, 1, 1, 0);
			//position.Transform(ToGLM(SelectedBone->PositionFrames[VisibleFrame]));
			//scale.Transform(ToGLM(SelectedBone->PositionFrames[VisibleFrame]));
			//ImGui::Text("Position: %.3f %.3f %.3f", position.X, position.Y, position.Z);
			//ImGui::Text("Scale: %.3f %.3f %.3f", scale.X, scale.Y, scale.Z);
			//ImGui::Text("Mass: %.5f %.5f %.5f %.5f %.5f %.5f",
			//	SelectedBone->Mass(0), SelectedBone->Mass(1), SelectedBone->Mass(2),
			//	SelectedBone->Mass(3), SelectedBone->Mass(4), SelectedBone->Mass(5));
			//vec3f l_vel = SelectedBone->vFrames[VisibleFrame];
			//ImGui::Text("Linear Velocity: %.3f %.3f %.3f", l_vel.X, l_vel.Y, l_vel.Z);
			//vec3f a_vel = SelectedBone->wFrames[VisibleFrame];
			//ImGui::Text("Angular Velocity: %.3f %.3f %.3f", a_vel.X, a_vel.Y, a_vel.Z);

			ImGui::End();
		}
	}
}

void CRigidDynamicsSimulation::Reset()
{
	SystemMutex.lock();
	for (SBone * Bone : Bones)
	{
		//Bone->PositionFrames.resize(1);
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	for (SBone * Bone : Bones)
	{
		Bone->SceneObject = new CSimpleMeshSceneObject();
		Bone->SceneObject->SetMesh(Application->CubeMesh);
		Bone->SceneObject->SetShader(Application->DiffuseShader);
		Bone->SceneObject->SetUniform("uColor", Bone->ColorUniform);
		Bone->ColorUniform = Bone->Color;
		Application->RenderPass->AddSceneObject(Bone->SceneObject);
	}
}

void CRigidDynamicsSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	VisibleFrame = CurrentFrame;

	SystemMutex.lock();
	for (SBone * Bone : Bones)
	{
		glm::mat4 Transformation = glm::mat4(1.f);
		Transformation = glm::scale(glm::mat4(1.f), (Bone->Extents * 2.f).ToGLM()) * Transformation;
		Transformation = glm::translate(glm::mat4(1.f), Bone->Anchor.ToGLM()) * Transformation;
		Transformation = glm::rotate(glm::mat4(1.f), Bone->RotationFrames[VisibleFrame].Z, glm::vec3(0, 0, 1)) * Transformation;
		Transformation = glm::rotate(glm::mat4(1.f), Bone->RotationFrames[VisibleFrame].Y, glm::vec3(0, 1, 0)) * Transformation;
		Transformation = glm::rotate(glm::mat4(1.f), Bone->RotationFrames[VisibleFrame].X, glm::vec3(1, 0, 0)) * Transformation;
		Transformation = glm::translate(glm::mat4(1.f), Bone->Position.ToGLM()) * Transformation;

		Bone->SceneObject->SetTransformation(Transformation);
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::PickParticle(ray3f const & Ray)
{
	SelectedBone = nullptr;
	for (SBone * Bone : Bones)
	{
		Bone->ColorUniform = Bone->Color;
	}
	for (SBone * Bone : Bones)
	{
		//vec3f Center;
		//Center.Transform(ToGLM(Bone->PositionFrames[VisibleFrame]), 1.f);
		//float const Radius = 0.125f;

		//if (Ray.IntersectsSphere(Center, Radius))
		//{
		//	SelectedParticle = Particle;
		//	Particle->ColorUniform = Colors::White * 0.75f;
		//	break;
		//}
	}
}
