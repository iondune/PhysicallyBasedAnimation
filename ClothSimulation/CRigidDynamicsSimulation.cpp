
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
	Transformation = glm::rotate(glm::mat4(1.f), Rotation.Z, glm::vec3(0, 0, 1)) * Transformation;
	Transformation = glm::rotate(glm::mat4(1.f), Rotation.Y, glm::vec3(0, 1, 0)) * Transformation;
	Transformation = glm::rotate(glm::mat4(1.f), Rotation.X, glm::vec3(1, 0, 0)) * Transformation;
	Transformation = glm::translate(glm::mat4(1.f), Translation.ToGLM()) * Transformation;
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

	SelectedBox = nullptr;
	for (SBox * const Box : Boxes)
	{
		if (Box->SceneObject)
		{
			Application->RenderPass->RemoveSceneObject(Box->SceneObject);
		}
		delete Box;
	}
	Boxes.clear();

	vec3d Center = Settings.Center;
	vec3d Size = Settings.Size;
	double const Density = 1e3;

	int n = 0;

	SBox * p = nullptr;

	p = new SBox();
	p->Extent = vec3d(0.3, 0.1, 0.1);
	p->Position = (RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center));
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = -1;
	p->Color = Colors::Blue;
	p->Fixed = true;
	Boxes.push_back(p);

	//n += 6;

	p = new SBox();
	p->Extent = vec3d(0.175, 0.075, 0.075);
	p->Position = (RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0.3 / 2 + 0.15 / 2 + 0.1, 0, 0)));
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
	p->Extent = vec3d(0.175, 0.075, 0.075);
	p->Position = (RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0.3 / 2 + 0.45 / 2 + 0.2, 0, 0)));
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
	p->Extent = vec3d(0.25, 0.25, 0.075);
	p->Position = (RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0, -0.1 - 0.1, 0)));
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = -1;
	p->Color = color3f(0.1f, 0.4f, 1.f);
	p->Fixed = true;
	Boxes.push_back(p);

	p = new SBox();
	p->Extent = vec3d(0.12);
	p->Position = (RotateAndTranslateToMatrix(vec3f(0, 0, 0), Center + vec3d(0, 0.1 + 0.04, 0)));
	p->m = Density * p->Extent.X * p->Extent.Y * p->Extent.Z;
	p->Mass(0) = (1.0 / 12.0) * p->m * Dot(p->Extent.YZ(), p->Extent.YZ());
	p->Mass(1) = (1.0 / 12.0) * p->m * Dot(p->Extent.XZ(), p->Extent.XZ());
	p->Mass(2) = (1.0 / 12.0) * p->m * Dot(p->Extent.XY(), p->Extent.XY());
	p->Mass(3) = p->m;
	p->Mass(4) = p->m;
	p->Mass(5) = p->m;
	p->Index = -1;
	p->Color = color3f(0.1f, 0.4f, 1.f);
	p->Fixed = true;
	Boxes.push_back(p);

	BodyMatrixSize = n;

	// Create planes
	SPlane Plane;
	Plane.Normal = vec2f(0.25f, 1.f).GetNormalized();
	Plane.Distance = -1.f;
	Planes.push_back(Plane);
	Plane.Normal = vec2f(-0.25f, 1.f).GetNormalized();
	Planes.push_back(Plane);


	// Create joints
	int j = 0;

	SJoint * joint;

	joint = new SJoint();
	joint->Body_i = Boxes[0];
	joint->Body_k = Boxes[1];
	joint->JointFrame = ToEigen(glm::translate(glm::mat4(1.f), glm::vec3(0.3f / 2 + 0.05, 0, 0)));
	joint->Index = j;
	Joints.push_back(joint);

	j += 3;

	joint = new SJoint();
	joint->Body_i = Boxes[1];
	joint->Body_k = Boxes[2];
	joint->JointFrame = ToEigen(glm::translate(glm::mat4(1.f), glm::vec3(0.15f / 2 + 0.05, 0, 0)));
	joint->Index = j;
	Joints.push_back(joint);

	j += 3;

	JointMatrixSize = j;

	AddSceneObjects();
	UpdateSceneObjects(0);

	for (int t = 0; t < Boxes.size(); ++ t)
	{
		cout << "Box " << t << " position: " << Boxes[t]->GetTranslation() << endl;
		Boxes[t]->OriginalTranslation = Boxes[t]->GetTranslation();
	}
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

	int const n = BodyMatrixSize;
	int const j = JointMatrixSize;
	double const h = TimeDelta;
	double const damping = 12.5;

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

	G.resize(j, n);
	G.setZero();

	struct SContactStore
	{
		vec3d Position;
		vec3d Normal;
		SBox * Which = nullptr;
	};

	vector<SContactStore> ContactsArray;

	for (SBox const * const Box : Boxes)
	{
		if (Box->Fixed)
		{
			continue;
		}

		Eigen::Matrix6d const M_i = Diagonal(Box->Mass);

		M.block<6, 6>(Box->Index, Box->Index) = M_i;

		Eigen::Vector6d const Phi_i = Box->GetPhi();

		v.segment(Box->Index, 6) = Phi_i;

		Eigen::Matrix6d Phi_i_cross;
		Phi_i_cross.setZero();
		Phi_i_cross.block<3, 3>(0, 0) = Rigid::bracket3(Phi_i.segment(0, 3));
		Phi_i_cross.block<3, 3>(3, 3) = Rigid::bracket3(Phi_i.segment(0, 3));

		Eigen::Vector3d const Acceleration = ToEigen(Gravity);
		Eigen::Matrix3d const Theta_i_T = ThetaFromE(Box->Position).transpose();

		Eigen::Vector3d const BodyForces = Theta_i_T * Box->m * Acceleration;
		Eigen::Vector6d const Coriolis = Phi_i_cross.transpose() * M_i * Phi_i;

		f.segment(Box->Index, 6) += Coriolis;
		f.segment(Box->Index + 3, 3) += BodyForces;
		f.segment(Box->Index, 3) += Box->AppliedTorque;
		f.segment(Box->Index + 3, 3) += Theta_i_T * Box->AppliedForce;
		f.segment(Box->Index + 3, 3) += Box->LocalForce;
		f.segment(Box->Index, 6) += Box->ReactionForce;

		// contacts
		
		Eigen::Matrix4d const E_i_k = Box->Position;
	}

	// joints
	for (SJoint const * const Joint : Joints)
	{
		Eigen::Matrix6d const Adjunct_ij = Rigid::adjoint(Joint->JointFrame.inverse());
		Eigen::Matrix6d const Adjunct_ki = Rigid::adjoint(Joint->Body_i->Position.inverse() * Joint->Body_k->Position);

		//cout << "Adjunct_ij = " << endl;
		//cout << Adjunct_ij << endl;
		//cout << endl;

		//cout << "Adjunct_ki = " << endl;
		//cout << Adjunct_ki << endl;
		//cout << endl;

		if (! Joint->Body_i->Fixed)
			G.block<3, 6>(Joint->Index, Joint->Body_i->Index) = Adjunct_ij.block<3, 6>(3, 0);
		if (! Joint->Body_k->Fixed)
			G.block<3, 6>(Joint->Index, Joint->Body_k->Index) = (-Adjunct_ij * Adjunct_ki).block<3, 6>(3, 0);

		//cout << "G = " << endl;
		//cout << G << endl;
		//cout << endl;

		//cout << "GT = " << endl;
		//cout << G.transpose() << endl;
		//cout << endl;
	}
	

	Eigen::MatrixXd const Mtilde = M + h*damping*M;
	Eigen::VectorXd const ftilde = M*v + h*f;

	Eigen::VectorXd NewV;
	Eigen::VectorXd ReactionForces;

	Eigen::MatrixXd A;
	A.resize(n + j, n + j);
	A.setZero();
	A.block(0, 0, n, n) = Mtilde;
	A.block(n, 0, j, n) = G;
	A.block(0, n, n, j) = G.transpose();

	Eigen::VectorXd b;
	b.resize(n + j);
	b.setZero();
	b.segment(0, n) = ftilde;

	Eigen::VectorXd const x = A.ldlt().solve(b);

	//cout << "x = " << endl;
	//cout << x << endl;
	//cout << endl;

	NewV = x.segment(0, n);
	ReactionForces = G.transpose() * x.segment(n, j);

	//cout << "ReactionForces = " << endl;
	//cout << ReactionForces << endl;
	//cout << endl;

	for (SBox * const Box : Boxes)
	{
		if (Box->Fixed)
		{
			Box->w = (0);
			Box->v = (0);
			Box->Position = (Box->Position);
			Box->ReactionForce = (Eigen::Vector6d::Zero());
			continue;
		}
		
		Eigen::Vector6d const Phi_i_k_1 = NewV.segment(Box->Index, 6);

		Eigen::Vector3d const w_k_1 = Phi_i_k_1.segment(0, 3);
		Eigen::Vector3d const v_k_1 = Phi_i_k_1.segment(3, 3);

		Eigen::Matrix4d E_i_k;
		E_i_k.setZero();
		E_i_k = Box->Position;

		Eigen::Matrix4d E_i_k_1;
		E_i_k_1.setZero();
		E_i_k_1 = Rigid::integrate(E_i_k, Phi_i_k_1, h);

		Box->w = (ToIon3D(w_k_1));
		Box->v = (ToIon3D(v_k_1));
		Box->Position = ((E_i_k_1));
		Box->ReactionForce = (ReactionForces.segment(Box->Index, 6));
		Box->AppliedForce = Eigen::Vector3d::Zero();
		Box->AppliedTorque = Eigen::Vector3d::Zero();
		Box->LocalForce = Eigen::Vector3d::Zero();
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

	if (SelectedBox)
	{
		if (ImGui::Begin("Rigid Body Info"))
		{
			ImGui::SetWindowSize(ImVec2(425, 250), ImGuiSetCond_Once);
			ImGui::SetWindowPos(ImVec2(1000, 350), ImGuiSetCond_Once);
			vec4f position(0, 0, 0, 1);
			vec4f scale(1, 1, 1, 0);
			position.Transform(ToGLM(SelectedBox->Position));
			scale.Transform(ToGLM(SelectedBox->Position));
			ImGui::Text("Position: %.3f %.3f %.3f", position.X, position.Y, position.Z);
			ImGui::Text("Scale: %.3f %.3f %.3f", scale.X, scale.Y, scale.Z);
			ImGui::Text("Mass: %.5f %.5f %.5f %.5f %.5f %.5f",
				SelectedBox->Mass(0), SelectedBox->Mass(1), SelectedBox->Mass(2),
				SelectedBox->Mass(3), SelectedBox->Mass(4), SelectedBox->Mass(5));
			vec3f l_vel = SelectedBox->v;
			ImGui::Text("Linear Velocity: %.3f %.3f %.3f", l_vel.X, l_vel.Y, l_vel.Z);
			vec3f a_vel = SelectedBox->w;
			ImGui::Text("Angular Velocity: %.3f %.3f %.3f", a_vel.X, a_vel.Y, a_vel.Z);

			ImGui::End();
		}
	}
}

void CRigidDynamicsSimulation::Reset()
{
}

void CRigidDynamicsSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	for (auto Box : Boxes)
	{
		Box->SceneObject = new CSimpleMeshSceneObject();
		Box->SceneObject->SetMesh(Application->CubeMesh);
		Box->SceneObject->SetScale(Box->Extent);
		Box->SceneObject->SetShader(Application->DiffuseShader);
		Box->SceneObject->SetUniform("uColor", Box->ColorUniform);
		Application->RenderPass->AddSceneObject(Box->SceneObject);
	}

	for (auto Joint : Joints)
	{
		Joint->SceneObject = new CSimpleMeshSceneObject();
		Joint->SceneObject->SetMesh(Application->SphereMesh);
		Joint->SceneObject->SetScale(0.025f);
		Joint->SceneObject->SetShader(Application->DiffuseShader);
		Joint->SceneObject->SetUniform("uColor", CUniform<color3f>(Colors::Black));
		Joint->SceneObject->SetFeatureEnabled(EDrawFeature::Wireframe, true);
		Application->RenderPass->AddSceneObject(Joint->SceneObject);

		Joint->CoordianteFrame = new CCoordinateFrameSceneObject();
		Joint->CoordianteFrame->SetScale(0.05f);
		Joint->CoordianteFrame->SetShader(Application->ColorShader);
		Application->RenderPass->AddSceneObject(Joint->CoordianteFrame);
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
	for (SBox * Box : Boxes)
	{
		Box->SceneObject->SetRotation(ToGLM(Box->Position));
		color3f const Color = Box->Color;
		Box->ColorUniform = Color;
	}

	for (auto Joint : Joints)
	{
		Joint->SceneObject->SetRotation(ToGLM(Joint->Body_i->Position * Joint->JointFrame));
		Joint->CoordianteFrame->SetRotation(ToGLM(Joint->Body_i->Position * Joint->JointFrame));
	}
}

void CRigidDynamicsSimulation::PickObject(ray3f const & Ray)
{
	SelectedBox = nullptr;
	for (auto Box : Boxes)
	{
		Box->ColorUniform = Box->Color;
	}
	for (auto Box : Boxes)
	{
		vec3f Center;
		Center.Transform(ToGLM(Box->Position), 1.f);
		float const Radius = 0.125f;

		if (Ray.IntersectsSphere(Center, Radius))
		{
			SelectedBox = Box;
			Box->ColorUniform = Colors::White * 0.75f;
			break;
		}
	}
}

Eigen::Vector6d CRigidDynamicsSimulation::SBox::GetPhi() const
{
	Eigen::Vector6d Phi;
	Phi(0) = w.X;
	Phi(1) = w.Y;
	Phi(2) = w.Z;
	Phi(3) = v.X;
	Phi(4) = v.Y;
	Phi(5) = v.Z;
	return Phi;
}

vec3d CRigidDynamicsSimulation::SBox::GetTranslation()
{
	vec3d p = 0;
	p.Transform(ToGLM(Position));
	return p;
}
