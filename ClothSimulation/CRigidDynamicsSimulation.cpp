
#include "CRigidDynamicsSimulation.h"
#include "CApplication.h"
#include "Util.h"
#include "SSparseMatrix.h"
#include "MosekSolver.h"
#include "odeBoxBox.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


Eigen::Matrix4d RotateAndTranslateToMatrix(vec3f const & Rotation, vec3f const & Translation)
{
	glm::mat4 Transformation = glm::mat4(1.f);
	Transformation = glm::rotate(Transformation, Rotation.Z, glm::vec3(0, 0, 1));
	Transformation = glm::rotate(Transformation, Rotation.Y, glm::vec3(0, 1, 0));
	Transformation = glm::rotate(Transformation, Rotation.X, glm::vec3(1, 0, 0));
	Transformation = glm::translate(Transformation, Translation.ToGLM());
	return ToEigen(Transformation);
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
	double const Density = 1e3;

	int n = 0;

	SBox * p = new SBox();
	p->Extent = Size;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center));
	p->wFrames.push_back(vec3d(1, 0, 0));
	p->vFrames.push_back(0);
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = n;
	Boxes.push_back(p);

	n += 6;

	p = new SBox();
	p->Extent = Size * 2;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0.2, 0, 0.4)));
	p->wFrames.push_back(vec3d(2, 0, 0));
	p->vFrames.push_back(vec3d(0, 1, 0));
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = n;
	p->Color = Colors::Green;
	Boxes.push_back(p);

	n += 6;

	p = new SBox();
	p->Extent = Size;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(-0.3, 0, 0)));
	p->wFrames.push_back(vec3d(5, 0, 5));
	p->vFrames.push_back(vec3d(0, 0, 1));
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = n;
	p->Color = Colors::Blue;
	Boxes.push_back(p);

	n += 6;

	p = new SBox();
	p->Extent = Size * 0.5;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0, 0.2, -0.5)));
	p->wFrames.push_back(vec3d(0, 0, 2));
	p->vFrames.push_back(vec3d(1, 0, 0));
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = n;
	p->Color = Colors::Magenta;
	Boxes.push_back(p);

	n += 6;


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

//Eigen::Matrix3d CrossProductMatrix(vec3d const & a)
//{
//	Eigen::Matrix3d M;
//	M.setZero();
//	M(1, 0) = a.Z;
//	M(2, 0) = -a.Y;
//	M(0, 1) = -a.Z;
//	M(2, 1) = a.X;
//	M(0, 2) = a.Y;
//	M(1, 2) = -a.X;
//	return M;
//}
//
//Eigen::Matrix3d CrossProductMatrix(Eigen::Vector3d const & a)
//{
//	Eigen::Matrix3d M;
//	M.setZero();
//	M(1, 0) = a.z();
//	M(2, 0) = -a.y();
//	M(0, 1) = -a.z();
//	M(2, 1) = a.x();
//	M(0, 2) = a.y();
//	M(1, 2) = -a.x();
//	return M;
//}

Eigen::Matrix4d ToEigen(glm::mat4 const & m)
{
	Eigen::Matrix4d M;
	M.setZero();
	for (int i = 0; i < 4; ++ i)
	{
		for (int j = 0; j < 4; ++ j)
		{
			M(i, j) = (double) m[j][i];
		}
	}
	return M;
}

glm::mat4 ToGLM(Eigen::Matrix4d const & m)
{
	glm::mat4 M;
	for (int i = 0; i < 4; ++ i)
	{
		for (int j = 0; j < 4; ++ j)
		{
			M[i][j] = (float) m(j, i);
		}
	}
	return M;
}

Eigen::Matrix3d ThetaFromE(Eigen::Matrix4d const & m)
{
	Eigen::Matrix3d M;
	M.setZero();
	for (int i = 0; i < 3; ++ i)
	{
		for (int j = 0; j < 3; ++ j)
		{
			M(i, j) = m(i, j);
		}
	}
	return M;
}

Eigen::Vector3d pFromE(Eigen::Matrix4d const & m)
{
	Eigen::Vector3d p;
	p.setZero();
	for (int i = 0; i < 3; ++ i)
	{
		p(i) = m(i, 3);
	}
	return p;
}

Eigen::Matrix6d Diagonal(Eigen::Vector6d const & v)
{
	Eigen::Matrix6d M;
	M.setIdentity();

	for (int i = 0; i < 6; ++ i)
	{
		M(i, i) = v(i);
	}

	return M;
}

void CRigidDynamicsSimulation::SimulateStep(double const TimeDelta)
{
	static vec3d const Gravity = vec3d(0, -9.8, 0);

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
	//SystemMutex.unlock();

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

	int const n = 6 * (int) Boxes.size();

	Eigen::MatrixXd M;
	Eigen::VectorXd f;
	Eigen::VectorXd v;

	M.resize(n, n);
	M.setZero();

	f.resize(n);
	f.setZero();

	v.resize(n);
	v.setZero();

	for (SBox * particle : Boxes)
	{
		SBox * box = particle;

		Eigen::Matrix6d const M_i = Diagonal(particle->Mass);

		M.block<6, 6>(box->Index, box->Index) = M_i;

		SystemMutex.lock();
		Eigen::Vector6d Phi_i_k;
		Phi_i_k.setZero();
		Phi_i_k(0) = particle->wFrames.back().X;
		Phi_i_k(1) = particle->wFrames.back().Y;
		Phi_i_k(2) = particle->wFrames.back().Z;
		Phi_i_k(3) = particle->vFrames.back().X;
		Phi_i_k(4) = particle->vFrames.back().Y;
		Phi_i_k(5) = particle->vFrames.back().Z;

		double const h = TimeDelta;

		Eigen::Matrix6d Phi_i_bracket_k;
		Phi_i_bracket_k.setZero();
		Phi_i_bracket_k.block<3, 3>(0, 0) = Rigid::bracket3(ToEigen(particle->wFrames.back()));
		Phi_i_bracket_k.block<3, 3>(3, 3) = Rigid::bracket3(ToEigen(particle->wFrames.back()));
		Phi_i_bracket_k.block<3, 3>(3, 0) = Rigid::bracket3(ToEigen(particle->vFrames.back()));

		Eigen::Vector3d const p_i = pFromE(particle->PositionFrames.back());
		Eigen::Vector3d const Acceleration = ToEigen(Gravity);
		Eigen::Matrix3d const Theta_i_T = ThetaFromE(particle->PositionFrames.back()).transpose();
		SystemMutex.unlock();

		Eigen::Vector6d B_E_i_k;
		B_E_i_k.setZero();
		B_E_i_k.segment(3, 3) = (Theta_i_T * particle->m * Acceleration);

		Eigen::Matrix6d const A = M_i;
		Eigen::Vector6d const b = M_i * Phi_i_k + h * (Phi_i_bracket_k.transpose() * M_i * Phi_i_k + B_E_i_k);

		//cout << "A =" << endl;
		//cout << A << endl;
		//cout << endl;
		//cout << "b =" << endl;
		//cout << b << endl;
		//cout << endl;

		Eigen::Vector6d const Phi_i_k_1 = A.ldlt().solve(b);

		Eigen::Vector3d const w_k_1 = Phi_i_k_1.segment(0, 3);
		Eigen::Vector3d const v_k_1 = Phi_i_k_1.segment(3, 3);

		SystemMutex.lock();
		Eigen::Matrix4d E_i_k;
		E_i_k.setZero();
		E_i_k = particle->PositionFrames.back();

		Eigen::Matrix4d E_i_k_1;
		E_i_k_1.setZero();
		E_i_k_1 = Rigid::integrate(E_i_k, Phi_i_k_1, h);

		particle->wFrames.push_back(ToIon3D(w_k_1));
		particle->vFrames.push_back(ToIon3D(v_k_1));
		particle->PositionFrames.push_back((E_i_k_1));

		Contacts const c = odeBoxBox(Eigen::Matrix4d::Identity(), ToEigen(vec3d(100, 0.5, 100)), E_i_k_1, ToEigen(particle->Extent));
		if (c.count > 0)
		{
		}
		SystemMutex.unlock();
	}
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
		if (ImGui::Begin("Rigid Body Info"))
		{
			ImGui::SetWindowSize(ImVec2(425, 250), ImGuiSetCond_Once);
			ImGui::SetWindowPos(ImVec2(1000, 350), ImGuiSetCond_Once);
			vec4f position(0, 0, 0, 1);
			vec4f scale(1, 1, 1, 0);
			position.Transform(ToGLM(SelectedParticle->PositionFrames[VisibleFrame]));
			scale.Transform(ToGLM(SelectedParticle->PositionFrames[VisibleFrame]));
			ImGui::Text("Position: %.3f %.3f %.3f", position.X, position.Y, position.Z);
			ImGui::Text("Scale: %.3f %.3f %.3f", scale.X, scale.Y, scale.Z);
			ImGui::Text("Mass: %.5f %.5f %.5f %.5f %.5f %.5f",
				SelectedParticle->Mass(0), SelectedParticle->Mass(1), SelectedParticle->Mass(2),
				SelectedParticle->Mass(3), SelectedParticle->Mass(4), SelectedParticle->Mass(5));
			vec3f l_vel = SelectedParticle->vFrames[VisibleFrame];
			ImGui::Text("Linear Velocity: %.3f %.3f %.3f", l_vel.X, l_vel.Y, l_vel.Z);
			vec3f a_vel = SelectedParticle->wFrames[VisibleFrame];
			ImGui::Text("Angular Velocity: %.3f %.3f %.3f", a_vel.X, a_vel.Y, a_vel.Z);

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
		Particle->SceneObject->SetUniform("uColor", CUniform<color3f>(Particle->Color));
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
		Particle->SceneObject->SetRotation(ToGLM(Particle->PositionFrames[CurrentFrame]));
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::PickParticle(ray3f const & Ray)
{
	SelectedParticle = nullptr;
	for (auto Particle : Boxes)
	{
		Particle->SceneObject->SetUniform("uColor", CUniform<color3f>(Particle->Color));
	}
	for (auto Particle : Boxes)
	{
		vec3f Center;
		Center.Transform(ToGLM(Particle->PositionFrames[VisibleFrame]), 1.f);
		float const Radius = 0.125f;

		if (Ray.IntersectsSphere(Center, Radius))
		{
			SelectedParticle = Particle;
			Particle->SceneObject->SetUniform("uColor", CUniform<color3f>(Colors::White * 0.75f));
			break;
		}
	}
}

CRigidDynamicsSimulation::SBox * CRigidDynamicsSimulation::GetParticle(vec2i const & Index)
{
	return Boxes[Index.X];
}
