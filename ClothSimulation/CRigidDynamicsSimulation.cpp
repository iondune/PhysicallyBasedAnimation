
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

Eigen::VectorXd quadprog(Eigen::MatrixXd const & H, Eigen::VectorXd const & f, Eigen::MatrixXd const & A, Eigen::VectorXd const & b, Eigen::VectorXd const & x0)
{
	//return H.ldlt().solve(f);
	return MosekSolver::Solve(H, f, A, b, x0);
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

	SBox * p = nullptr;

	p = new SBox();
	p->Extent = Size;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center));
	p->wFrames.push_back(vec3d(1, 0, 0));
	p->vFrames.push_back(0);
	p->contactFrames.push_back(vector<Contacts>());
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = n;
	p->Color = Colors::Red;
	Boxes.push_back(p);

	n += 6;

	p = new SBox();
	p->Extent = Size * 2;
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0.175, 0, 0)));
	p->wFrames.push_back(vec3d(2, 0, 0));
	p->vFrames.push_back(vec3d(0, 1, 0));
	p->contactFrames.push_back(vector<Contacts>());
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
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(-0.2, 0, 0)));
	p->wFrames.push_back(vec3d(5, 0, 5));
	p->vFrames.push_back(vec3d(0, 0, 1));
	p->contactFrames.push_back(vector<Contacts>());
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
	p->PositionFrames.push_back(RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(-0.1, 0, 0)));
	p->wFrames.push_back(vec3d(0, 0, 2));
	p->vFrames.push_back(vec3d(1, 0, 0));
	p->contactFrames.push_back(vector<Contacts>());
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

	int const n = 6 * (int) Boxes.size();
	double const h = TimeDelta;
	double const damping = 0.7;
	double const restitution = 0.2;

	Eigen::MatrixXd M;
	Eigen::VectorXd f;
	Eigen::VectorXd v;
	Eigen::MatrixXd G;

	M.resize(n, n);
	M.setZero();

	f.resize(n);
	f.setZero();

	v.resize(n);
	v.setZero();

	G.resize(n, n);
	G.setZero();

	struct SContactStore
	{
		vec3d Position;
		vec3d Normal;
		SBox * Which = nullptr;
	};

	vector<SContactStore> ContactsArray;

	SystemMutex.lock();
	for (SBox * const Box : Boxes)
	{
		Eigen::Matrix6d const M_i = Diagonal(Box->Mass);

		M.block<6, 6>(Box->Index, Box->Index) = M_i;

		Eigen::Vector6d Phi_i_k = Box->GetPhi();
		//Phi_i_k.setZero();
		//Phi_i_k(0) = Box->wFrames.back().X;
		//Phi_i_k(1) = Box->wFrames.back().Y;
		//Phi_i_k(2) = Box->wFrames.back().Z;
		//Phi_i_k(3) = Box->vFrames.back().X;
		//Phi_i_k(4) = Box->vFrames.back().Y;
		//Phi_i_k(5) = Box->vFrames.back().Z;

		v.segment(Box->Index, 6) = Phi_i_k;

		Eigen::Matrix6d Phi_i_bracket_k;
		Phi_i_bracket_k.setZero();
		Phi_i_bracket_k.block<3, 3>(0, 0) = Rigid::bracket3(ToEigen(Box->wFrames.back()));
		Phi_i_bracket_k.block<3, 3>(3, 3) = Rigid::bracket3(ToEigen(Box->wFrames.back()));
		Phi_i_bracket_k.block<3, 3>(3, 0) = Rigid::bracket3(ToEigen(Box->vFrames.back()));

		Eigen::Vector3d const Acceleration = ToEigen(Gravity);
		Eigen::Matrix3d const Theta_i_T = ThetaFromE(Box->PositionFrames.back()).transpose();

		Eigen::Vector3d const BodyForces = Theta_i_T * Box->m * Acceleration;
		Eigen::Vector6d const Coriolis = Phi_i_bracket_k.transpose() * M_i * Phi_i_k;

		f.segment(Box->Index, 6) += Coriolis;
		f.segment(Box->Index + 3, 3) += BodyForces;

		// contacts
		
		Eigen::Matrix4d const E_i_k = Box->PositionFrames.back();

		SingletonPointer<CApplication> Application;

		Box->contactFrames.push_back(vector<Contacts>());
		glm::mat4 FloorMatrix = glm::mat4(1.f);
		FloorMatrix = glm::translate(FloorMatrix, Application->GroundObject->GetTranslation().ToGLM());

		Contacts const c = odeBoxBox(ToEigen(FloorMatrix), ToEigen(vec3d(1.0) * Application->GroundObject->GetScale()), E_i_k, ToEigen(Box->Extent));
		if (c.count > 0)
		{
			Box->contactFrames.back().push_back(c);
			for (int i = 0; i < c.count; ++ i)
			{
				SContactStore Store;
				Store.Which = Box;
				Store.Normal = ToIon3D(c.normal);
				Store.Position = ToIon3D(c.positions[i]);
				//ContactsArray.push_back(Store);
			}
		}
	}

	// joints
	SBox * body_i = Boxes[0];
	SBox * body_j = Boxes[1];
	Eigen::Matrix4d JointFrame = ToEigen(glm::translate(glm::mat4(1.f), glm::vec3(0.11f, 0, 0)));
	
	Eigen::Vector6d deltaPhi = (Rigid::adjoint(JointFrame) * body_i->GetPhi() - Rigid::adjoint(JointFrame) * Rigid::adjoint(body_i->PositionFrames.back().inverse() * body_j->PositionFrames.back()) * body_j->GetPhi());
	deltaPhi(0) = 0;
	deltaPhi(1) = 0;
	deltaPhi(2) = 0;

	cout << "deltaPhi = " << endl;
	cout << deltaPhi << endl;
	cout << endl;

	//G.block<6, 6>(body_j->Index, body_j->Index) = Diagonal(deltaPhi);

	SystemMutex.unlock();


	Eigen::MatrixXd const Mtilde = M + h*damping*M;
	Eigen::VectorXd const ftilde = M*v + h*f;

	Eigen::VectorXd NewV;
	if (ContactsArray.size())
	{
		Eigen::MatrixXd ContactMatrix;
		ContactMatrix.resize(ContactsArray.size(), n);
		ContactMatrix.setZero();

		for (int i = 0; i < ContactsArray.size(); ++ i)
		{
			auto C = ContactsArray[i];
			Eigen::Vector3d const nw = ToEigen(C.Normal);
			Eigen::Matrix4d const E = C.Which->PositionFrames.back();
			glm::mat4 const m = ToGLM(E);
			vec3d const local = C.Position.GetTransformed(glm::inverse(m));

			Eigen::Matrix<double, 1, 6> const ContactRow = nw.transpose() * ThetaFromE(C.Which->PositionFrames.back()) * Rigid::gamma(ToEigen(local));

			ContactMatrix.block<1, 6>(i, C.Which->Index) = ContactRow;
		}

		Eigen::VectorXd Nv;
		Nv.resize(ContactsArray.size());
		Nv = restitution * ContactMatrix * v;

		Eigen::VectorXd LinV = Mtilde.ldlt().solve(ftilde);
		NewV = quadprog(Mtilde, -ftilde, -ContactMatrix, Nv, v);
		cout << "Linear Solution=" << endl << LinV << endl;
		cout << "QP Solution=" << endl << NewV << endl;
	}
	else
	{
		//NewV = Mtilde.ldlt().solve(ftilde);

		Eigen::MatrixXd A;
		A.resize(n * 2, n * 2);
		A.setZero();
		A.block(0, 0, n, n) = Mtilde;
		A.block(n, 0, n, n) = G;
		A.block(0, n, n, n) = G.transpose();

		Eigen::VectorXd b;
		b.resize(n * 2);
		b.segment(0, n) = ftilde;

		Eigen::VectorXd x = A.ldlt().solve(b);
		NewV = x.segment(0, n);
	}

	SystemMutex.lock();
	for (SBox * const Box : Boxes)
	{
		Eigen::Vector6d const Phi_i_k_1 = NewV.segment(Box->Index, 6);

		Eigen::Vector3d const w_k_1 = Phi_i_k_1.segment(0, 3);
		Eigen::Vector3d const v_k_1 = Phi_i_k_1.segment(3, 3);

		Eigen::Matrix4d E_i_k;
		E_i_k.setZero();
		E_i_k = Box->PositionFrames.back();

		Eigen::Matrix4d E_i_k_1;
		E_i_k_1.setZero();
		E_i_k_1 = Rigid::integrate(E_i_k, Phi_i_k_1, h);

		Box->wFrames.push_back(ToIon3D(w_k_1));
		Box->vFrames.push_back(ToIon3D(v_k_1));
		Box->PositionFrames.push_back((E_i_k_1));
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

			ImGui::Separator();

			for (auto Contact : SelectedParticle->contactFrames[VisibleFrame])
			{
				ImGui::Text("Contacts %d", Contact.count);
				ImGui::Text("Normal %.3f %.3f %.3f", Contact.normal.x(), Contact.normal.y(), Contact.normal.z());
				ImGui::Separator();
			}

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
		Particle->SceneObject->SetScale(Particle->Extent);
		Particle->SceneObject->SetShader(Application->DiffuseShader);
		Particle->SceneObject->SetUniform("uColor", Particle->ColorUniform);
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
			//Application->RenderPass->AddSceneObject(PlaneObject);
		}

		PlaneObjectsCreated = true;
	}
}

void CRigidDynamicsSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	VisibleFrame = CurrentFrame;

	SystemMutex.lock();
	for (SBox * Box : Boxes)
	{
		Box->SceneObject->SetRotation(ToGLM(Box->PositionFrames[CurrentFrame]));
		color3f Color = Box->Color;
		if (Box->contactFrames[CurrentFrame].size())
			Color *= 0.5f;
		Box->ColorUniform = Color;
	}
	SystemMutex.unlock();
}

void CRigidDynamicsSimulation::PickParticle(ray3f const & Ray)
{
	SelectedParticle = nullptr;
	for (auto Particle : Boxes)
	{
		Particle->ColorUniform = Particle->Color;
	}
	for (auto Particle : Boxes)
	{
		vec3f Center;
		Center.Transform(ToGLM(Particle->PositionFrames[VisibleFrame]), 1.f);
		float const Radius = 0.125f;

		if (Ray.IntersectsSphere(Center, Radius))
		{
			SelectedParticle = Particle;
			Particle->ColorUniform = Colors::White * 0.75f;
			break;
		}
	}
}

CRigidDynamicsSimulation::SBox * CRigidDynamicsSimulation::GetParticle(vec2i const & Index)
{
	return Boxes[Index.X];
}

Eigen::Vector6d CRigidDynamicsSimulation::SBox::GetPhi() const
{
	Eigen::Vector6d Phi;
	Phi(0) = wFrames.back().X;
	Phi(1) = wFrames.back().Y;
	Phi(2) = wFrames.back().Z;
	Phi(3) = vFrames.back().X;
	Phi(4) = vFrames.back().Y;
	Phi(5) = vFrames.back().Z;
	return Phi;
}
