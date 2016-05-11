
#include "CRigidDynamicsSimulation.h"
#include "CApplication.h"
#include "Util.h"
#include "SSparseMatrix.h"
#include "MosekSolver.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Sparse>

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


glm::mat4 RotateAndTranslateToMatrix(vec3f const & Rotation, vec3f const & Translation)
{
	glm::mat4 Transformation = glm::rotate(glm::mat4(1.f), Rotation.Z, glm::vec3(0, 0, 1));
	Transformation = glm::rotate(Transformation, Rotation.Y, glm::vec3(0, 1, 0));
	Transformation = glm::rotate(Transformation, Rotation.X, glm::vec3(1, 0, 0));
	Transformation = glm::translate(Transformation, Translation.ToGLM());
	return Transformation;
}

CRigidDynamicsSimulation::CRigidDynamicsSimulation()
{}

void CRigidDynamicsSimulation::Setup()
{
	SingletonPointer<CApplication> Application;

	SelectedParticle = nullptr;
	for (auto Particle : Boxes)
	{
		if (Particle->SceneObject)
		{
			Application->RenderPass->RemoveSceneObject(Particle->SceneObject);
		}
		delete Particle;
	}
	Boxes.clear();

	vec3d Center = Settings.Center;
	vec3d Size = Settings.Size;

	SBox * p = new SBox();
	p->Extent = Size;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center));
	p->Mass = 0.2f;
	Boxes.push_back(p);


	// Create planes
	SPlane Plane;
	Plane.Normal = vec2f(0.25f, 1.f).GetNormalized();
	Plane.Distance = -1.f;
	Planes.push_back(Plane);
	Plane.Normal = vec2f(-0.25f, 1.f).GetNormalized();
	Planes.push_back(Plane);

	AddSceneObjects();
	UpdateSceneObjects(0);
}

void CRigidDynamicsSimulation::SimulateStep(double const TimeDelta)
{
	static vec3f const Gravity = vec3f(0, -9.8f, 0);

	/*Eigen::VectorXd v;
	v.resize(3);
	v.setZero();

	Eigen::VectorXd f;
	f.resize(3);
	f.setZero();

	SystemMutex.lock();
	for (SParticle * particle : Particles)
	{
		if (! particle->IsFixed)
		{
			f.segment(particle->Index, 2) = ToEigen(Gravity * particle->Mass);
			v.segment(particle->Index, 2) = ToEigen(particle->VelocityFrames.back());
			M.Set(particle->Index, particle->Index, particle->Mass);
			M.Set(particle->Index + 1, particle->Index + 1, particle->Mass);
		}
	}

	for (SSpring * spring : Springs)
	{
		vec2d const PositionDelta = spring->Particle1->PositionFrames.back() - spring->Particle0->PositionFrames.back();
		double const CurrentLength = PositionDelta.Length();
		vec2d const SpringForce = spring->Stiffness * (CurrentLength - spring->RestLength) * PositionDelta / CurrentLength;

		Eigen::Matrix2d const I = Eigen::Matrix2d::Identity();
		Eigen::Matrix2d const StiffnessMatrix = (spring->Stiffness / Sq(CurrentLength)) * (
			(1 - (CurrentLength - spring->RestLength) / CurrentLength) * (ToEigen(PositionDelta) * ToEigen(PositionDelta).transpose()) +
			((CurrentLength - spring->RestLength) / CurrentLength) * (Dot(PositionDelta, PositionDelta) * I)
			);

		if (! spring->Particle0->IsFixed)
		{
			f.segment(spring->Particle0->Index, 2) += ToEigen(SpringForce);
			K.Add(spring->Particle0->Index, spring->Particle0->Index, StiffnessMatrix);
		}
		if (! spring->Particle1->IsFixed)
		{
			f.segment(spring->Particle1->Index, 2) -= ToEigen(SpringForce);
			K.Add(spring->Particle1->Index, spring->Particle1->Index, StiffnessMatrix);
		}
		if (! spring->Particle0->IsFixed && ! spring->Particle1->IsFixed)
		{
			K.Subtract(spring->Particle0->Index, spring->Particle1->Index, StiffnessMatrix);
			K.Subtract(spring->Particle1->Index, spring->Particle0->Index, StiffnessMatrix);
		}
	}*/

	uint NumCollisions = 0;
	vector<vector<double>> CollisionMatrix;

	//for (SParticle * particle : Particles)
	//{
	//	if (! particle->IsFixed)
	//	{
	//		if (particle->PositionFrames.back().Y < -0.5)
	//		{
	//			CollisionMatrix.push_back(vector<double>());
	//			CollisionMatrix.back().resize(MatrixSize, 0.0);

	//			// Collision Normal
	//			CollisionMatrix[NumCollisions][particle->Index + 0] = 0.0;
	//			CollisionMatrix[NumCollisions][particle->Index + 1] = 1.0;

	//			NumCollisions ++;
	//		}
	//	}
	//}
	SystemMutex.unlock();

	//SSparseMatrix const D = Damping.X * TimeDelta * M + Damping.Y * Sq(TimeDelta) * K;

	Eigen::VectorXd Result;
	if (NumCollisions == 0)
	{
		//SSparseMatrix const A = M + D;
		//Eigen::VectorXd const b = M.ToEigen() * v + TimeDelta * f;
		//Eigen::SparseMatrix<double> const AEigen = A.ToEigen();

		//Eigen::ConjugateGradient< Eigen::SparseMatrix<double> > cg;
		//cg.setMaxIterations(25);
		//cg.setTolerance(1e-3);
		//cg.compute(AEigen);

		//cout << "A =" << endl;
		//cout << ASparse << endl;
		//cout << endl;
		//cout << "b =" << endl;
		//cout << b << endl;
		//cout << endl;

		//Result = cg.solveWithGuess(b, v);
	}
	else
	{
		//SSparseMatrix const Mtilde = M + Sq(TimeDelta) * K + TimeDelta * D;
		//Eigen::VectorXd const fTilde = M * v + TimeDelta * f;

		//Result = MosekSolver::Solve(Mtilde, fTilde, CollisionMatrix);
	}

	//cout << "x =" << endl;
	//cout << Result << endl;
	//cout << endl;

	SystemMutex.lock();
	for (SBox * particle : Boxes)
	{
		//if (! particle->IsFixed)
		//{
		//	particle->VelocityFrames.push_back(ToIon2D(Result.segment(particle->Index, 2)));

		//	//if (particle->ConstraintType == EConstraintType::XAxis)
		//	//{
		//	//	particle->VelocityFrames.back() *= vec2d(1, 0);
		//	//}
		//	//else if (particle->ConstraintType == EConstraintType::YAxis)
		//	//{
		//	//	particle->VelocityFrames.back() *= vec2d(0, 1);
		//	//}
		//	//else if (particle->ConstraintType == EConstraintType::DownDiagonal)
		//	//{
		//	//	vec2d const Vector = vec2d(1, -1);
		//	//	particle->VelocityFrames.back() = Dot(particle->VelocityFrames.back(), Vector) * Vector.GetNormalized();
		//	//}
		//	//else if (particle->ConstraintType == EConstraintType::UpDiagonal)
		//	//{
		//	//	vec2d const Vector = vec2d(1, 1);
		//	//	particle->VelocityFrames.back() = Dot(particle->VelocityFrames.back(), Vector) * Vector.GetNormalized();
		//	//}

		//	particle->PositionFrames.push_back(particle->PositionFrames.back() + TimeDelta * particle->VelocityFrames.back());

		//	for (size_t i = 0; i < Planes.size(); ++ i)
		//	{
		//		SPlane const & Plane = Planes[i];

		//		double const Distance = Dot(particle->PositionFrames.back(), Plane.Normal);

		//		if (Distance < Plane.Distance)
		//		{
		//			particle->PositionFrames.back() += Plane.Normal * (Plane.Distance - Distance);
		//			particle->VelocityFrames.back() -= Plane.Normal * Dot(particle->VelocityFrames.back().GetNormalized(), Plane.Normal);
		//		}
		//	}

		//}
		//else
		//{
		//	particle->VelocityFrames.push_back(0);
		//	particle->PositionFrames.push_back(particle->PositionFrames.back());
		//}
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::GUI()
{
	SingletonPointer<CSimulationSystem> SimulationSystem;

	if (ImGui::BeginPopupModal("Cloth Settings"))
	{
		ImGui::SetWindowSize(ImVec2(500, 350), ImGuiSetCond_Once);

		float Size[3] = { (float) Settings.Size.X, (float) Settings.Size.Y, (float) Settings.Size.Z };
		if (ImGui::SliderFloat2("Size", Size, 0.1f, 2.5f))
		{
			Settings.Size.X = Size[0];
			Settings.Size.Y = Size[1];
			SimulationSystem->Reset();
			Setup();
		}

		float Center[3] = { (float) Settings.Center.X, (float) Settings.Center.Y };
		if (ImGui::SliderFloat2("Center", Center, -2.f, 2.f))
		{
			Settings.Center.X = Center[0];
			Settings.Center.Y = Center[1];
			SimulationSystem->Reset();
			Setup();
		}

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

	if (SelectedParticle)
	{
		if (ImGui::Begin("Edit Cloth Node"))
		{
			ImGui::SetWindowSize(ImVec2(350, 150), ImGuiSetCond_Once);
			ImGui::SetWindowPos(ImVec2(1000, 350), ImGuiSetCond_Once);
			//ImGui::Text("Position: %.3f %.3f", SelectedParticle->PositionFrames[VisibleFrame].X, SelectedParticle->PositionFrames[VisibleFrame].Y);

			ImGui::End();
		}
	}
}

void CRigidDynamicsSimulation::Reset()
{
	SystemMutex.lock();
	for (SBox * particle : Boxes)
	{
		particle->PositionFrames.resize(1);
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	for (auto Particle : Boxes)
	{
		Particle->SceneObject = new CSimpleMeshSceneObject();
		Particle->SceneObject->SetMesh(Application->CubeMesh);
		Particle->SceneObject->SetScale(Particle->Extent * 2);
		Particle->SceneObject->SetShader(Application->DiffuseShader);
		Particle->SceneObject->SetUniform("uColor", CUniform<color3f>(Colors::Red));
		Application->RenderPass->AddSceneObject(Particle->SceneObject);
	}

	if (! PlaneObjectsCreated)
	{
		for (size_t i = 0; i < Planes.size(); ++ i)
		{
			SPlane const & Plane = Planes[i];

			CSimpleMeshSceneObject * PlaneObject = new CSimpleMeshSceneObject();
			PlaneObject->SetMesh(Application->CubeMesh);
			PlaneObject->SetShader(Application->DiffuseShader);
			PlaneObject->SetUniform("uColor", CUniform<color3f>(i ? Colors::Blue : Colors::Green));
			PlaneObject->SetScale(vec3f(2.5f, 0.05f, 1.f + 0.1f * i));
			double const Rotation = -atan2(Plane.Normal.X, Plane.Normal.Y);
			PlaneObject->SetRotation(vec3f(0, 0, (float) Rotation));
			PlaneObject->SetPosition(Plane.Normal * (Plane.Distance - 0.025f));
			Application->RenderPass->AddSceneObject(PlaneObject);
		}

		PlaneObjectsCreated = true;
	}
}

void CRigidDynamicsSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	VisibleFrame = CurrentFrame;

	SystemMutex.lock();
	for (auto Particle : Boxes)
	{
		Particle->SceneObject->SetRotation(Particle->PositionFrames[CurrentFrame]);
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::PickParticle(ray3f const & Ray)
{
	SelectedParticle = nullptr;
	for (auto Particle : Boxes)
	{
		Particle->SceneObject->SetUniform("uColor", CUniform<color3f>(Colors::Red));
	}
	for (auto Particle : Boxes)
	{
		//vec3f const Center = Particle->PositionFrames[VisibleFrame];
		//float const Radius = 0.025f;

		//if (Ray.IntersectsSphere(Center, Radius))
		{
			SelectedParticle = Particle;
			Particle->SceneObject->SetUniform("uColor", CUniform<color3f>(Colors::Yellow));
			break;
		}
	}
}

CRigidDynamicsSimulation::SBox * CRigidDynamicsSimulation::GetParticle(vec2i const & Index)
{
	return Boxes[Index.X];
}
