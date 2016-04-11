
#include "CClothSimulation.h"
#include "CApplication.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Sparse>

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


CClothSimulation::CClothSimulation()
{}

void CClothSimulation::Setup()
{
	Particles.clear();
	Springs.clear();

	vec2d x00(-0.25, 0.5);
	vec2d x01(0.25, 0.5);
	vec2d x10(-0.25, 0);
	vec2d x11(0.25, 0);

	assert(Settings.rows > 1);
	assert(Settings.cols > 1);
	assert(Settings.mass > 0.0);
	assert(Settings.stiffness > 0.0);

	Rows = Settings.rows;
	Columns = Settings.cols;
	Damping = Settings.damping;

	// Create particles
	double r = 0.02; // Used for collisions
	int nVerts = Rows*Columns;
	for (int i = 0; i < Rows; ++i)
	{
		double u = i / (Rows - 1.0);
		vec2d x0 = (1 - u)*x00 + u*x10;
		vec2d x1 = (1 - u)*x01 + u*x11;
		for (int j = 0; j < Columns; ++j)
		{
			SParticle * p = new SParticle();
			Particles.push_back(p);

			p->Radius = r;
			double v = j / (Columns - 1.0);
			p->PositionFrames.push_back((1 - v)*x0 + v*x1);
			p->VelocityFrames.push_back(0.0);
			p->Mass = Settings.mass / (nVerts);

			if (i == 0 && j == 0)
			{
				p->IsConstrained = true;
			}

			p->IsFixed = false;
			p->Index = MatrixSize;
			MatrixSize += 2;
		}
	}

	// Create x springs
	for (int i = 0; i < Rows; ++i) {
		for (int j = 0; j < Columns - 1; ++j) {
			int k0 = i*Columns + j;
			int k1 = k0 + 1;
			Springs.push_back(new SSpring(Particles[k0], Particles[k1], Settings.stiffness));
		}
	}

	// Create y springs
	for (int j = 0; j < Columns; ++j) {
		for (int i = 0; i < Rows - 1; ++i) {
			int k0 = i*Columns + j;
			int k1 = k0 + Columns;
			Springs.push_back(new SSpring(Particles[k0], Particles[k1], Settings.stiffness));
		}
	}

	// Create shear springs
	for (int i = 0; i < Rows - 1; ++i) {
		for (int j = 0; j < Columns - 1; ++j) {
			int k00 = i*Columns + j;
			int k10 = k00 + 1;
			int k01 = k00 + Columns;
			int k11 = k01 + 1;
			Springs.push_back(new SSpring(Particles[k00], Particles[k11], Settings.stiffness));
			Springs.push_back(new SSpring(Particles[k10], Particles[k01], Settings.stiffness));
		}
	}

	AddSceneObjects();
	UpdateSceneObjects(0);
}

Eigen::Vector3d ToEigen(vec3d const & v)
{
	Eigen::Vector3d ret;
	ret << v.X, v.Y, v.Z;
	return ret;
}

Eigen::Vector2d ToEigen(vec2d const & v)
{
	Eigen::Vector2d ret;
	ret << v.X, v.Y;
	return ret;
}

Eigen::Array<double, 1, 1> ToEigen(double const x)
{
	Eigen::Array<double, 1, 1> ret;
	ret << x;
	return ret;
}

vec3d ToIon3D(Eigen::Vector3d const & v)
{
	return vec3d(v.x(), v.y(), v.z());
}

vec2d ToIon2D(Eigen::Vector2d const & v)
{
	return vec2d(v.x(), v.y());
}

struct SSparseMatrix
{
	map<vec2i, double> Elements;

	void Set(int const x, int const y, double const Value)
	{
		Elements[vec2i(x, y)] = Value;
	}

	void Add(int const x0, int const y0, Eigen::Matrix2d const & Mat)
	{
		for (int y = 0; y < 2; ++ y)
		{
			for (int x = 0; x < 2; ++ x)
			{
				Elements[vec2i(x + x0, y + y0)] += Mat(x, y);
			}
		}
	}

	void Subtract(int const x0, int const y0, Eigen::Matrix2d const & Mat)
	{
		for (int y = 0; y < 2; ++ y)
		{
			for (int x = 0; x < 2; ++ x)
			{
				Elements[vec2i(x + x0, y + y0)] -= Mat(x, y);
			}
		}
	}

	SSparseMatrix operator + (SSparseMatrix const & other) const
	{
		SSparseMatrix ret = *this;

		for (auto & Element : other.Elements)
		{
			ret.Elements[Element.first] += Element.second;
		}

		return ret;
	}

	friend SSparseMatrix operator * (double const lhs, SSparseMatrix const & rhs)
	{
		SSparseMatrix ret = rhs;

		for (auto & Element : ret.Elements)
		{
			Element.second *= lhs;
		}

		return ret;
	}

	Eigen::SparseMatrix<double> Get(int const Width, int const Height) const
	{
		std::vector<Eigen::Triplet<double>> Tuples;

		for (auto const & Element : Elements)
		{
			Tuples.push_back(Eigen::Triplet<double>(Element.first.X, Element.first.Y, Element.second));

			if (Element.first.X + 1 > Width || Element.first.Y + 1 > Height)
			{
				Log::Warn("Out-of-bounds element in sparse matrix.");
			}
		}

		Eigen::SparseMatrix<double> A(Width, Height);
		A.setFromTriplets(Tuples.begin(), Tuples.end());
		return A;
	}
};

void CClothSimulation::SimulateStep(double const TimeDelta)
{
	static vec2d const Gravity = vec2d(0, -9.8);

	SSparseMatrix M;
	SSparseMatrix K;

	Eigen::VectorXd v;
	v.resize(MatrixSize);
	v.setZero();

	Eigen::VectorXd f;
	f.resize(MatrixSize);
	f.setZero();

	ParticlesMutex.lock();
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
	}
	ParticlesMutex.unlock();

	SSparseMatrix const D = Damping.X * TimeDelta * M + Damping.Y * Sq(TimeDelta) * K;

	SSparseMatrix const A = M + D;
	Eigen::SparseMatrix<double> const ASparse = A.Get(MatrixSize, MatrixSize);
	Eigen::VectorXd const b = M.Get(MatrixSize, MatrixSize) * v + TimeDelta * f;
	
	Eigen::ConjugateGradient< Eigen::SparseMatrix<double> > cg;
	cg.setMaxIterations(25);
	cg.setTolerance(1e-3);
	cg.compute(ASparse);

	//cout << "A =" << endl;
	//cout << ASparse << endl;
	//cout << endl;
	//cout << "b =" << endl;
	//cout << b << endl;
	//cout << endl;
	Eigen::VectorXd Result = cg.solveWithGuess(b, v);
	//cout << "x =" << endl;
	//cout << Result << endl;
	//cout << endl;

	ParticlesMutex.lock();
	for (SParticle * particle : Particles)
	{
		if (! particle->IsFixed)
		{
			particle->VelocityFrames.push_back(ToIon2D(Result.segment(particle->Index, 2)));

			if (particle->IsConstrained)
			{
				particle->VelocityFrames.back() *= vec2d(1, 0);
			}

			particle->PositionFrames.push_back(particle->PositionFrames.back() + TimeDelta * particle->VelocityFrames.back());
		}
		else
		{
			particle->VelocityFrames.push_back(0);
			particle->PositionFrames.push_back(particle->PositionFrames.back());
		}
	}

	ParticlesMutex.unlock();
}

void CClothSimulation::GUI()
{
	SingletonPointer<CSimulationSystem> SimulationSystem;

	if (ImGui::BeginPopupModal("Cloth Settings"))
	{
		ImGui::SetWindowSize(ImVec2(500, 350));
		int Rows = Settings.rows;
		if (ImGui::SliderInt("Rows", &Rows, 2, 20))
		{
			Settings.rows = Rows;
			Setup();
			SimulationSystem->Reset();
		}

		int Columns = Settings.cols;
		if (ImGui::SliderInt("Columns", &Columns, 2, 20))
		{
			Settings.cols = Columns;
			Setup();
			SimulationSystem->Reset();
		}

		float Mass = (float) Settings.mass;
		if (ImGui::InputFloat("Mass", &Mass))
		{
			Settings.mass = (double) Mass;
			Setup();
			SimulationSystem->Reset();
		}

		float Stiffness = (float) Settings.stiffness;
		if (ImGui::InputFloat("Stiffness", &Mass))
		{
			Settings.stiffness = (double) Stiffness;
			Setup();
			SimulationSystem->Reset();
		}

		vec2d damping = vec2d(0.0, 1.0);

		if (ImGui::Button("Apply"))
		{
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

void CClothSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	if (ClothMesh)
	{
		ClothMesh->Clear();
	}
	else
	{
		ClothMesh = new CSimpleMesh();
	}

	for (int x = 0; x < Rows - 1; ++ x)
	{
		for (int y = 0; y < Columns - 1; ++ y)
		{
			uint const Start = (uint) ClothMesh->Vertices.size();

			CSimpleMesh::SVertex Vertex;
			for (int i = 0; i < 4; ++ i)
			{
				ClothMesh->Vertices.push_back(Vertex);
			}

			CSimpleMesh::STriangle Triangle;
			Triangle.Indices[0] = Start + 0;
			Triangle.Indices[1] = Start + 2;
			Triangle.Indices[2] = Start + 1;
			ClothMesh->Triangles.push_back(Triangle);

			Triangle.Indices[0] = Start + 0;
			Triangle.Indices[1] = Start + 3;
			Triangle.Indices[2] = Start + 2;
			ClothMesh->Triangles.push_back(Triangle);
		}
	}

	if (! ClothObjectFront)
	{
		ClothObjectFront = new CSimpleMeshSceneObject();
		ClothObjectFront->SetShader(Application->ClothShader);
		ClothObjectFront->SetUniform("uColor", CUniform<color3f>(Colors::Red));
		ClothObjectFront->SetUniform("uFlipNormals", CUniform<int>(0));
		ClothObjectFront->SetFeatureEnabled(EDrawFeature::CullBack, true);
		ClothObjectFront->SetFeatureEnabled(EDrawFeature::Wireframe, true);
		Application->RenderPass->AddSceneObject(ClothObjectFront);
	}
	ClothObjectFront->SetMesh(ClothMesh);

	if (! ClothObjectBack)
	{
		ClothObjectBack = new CSimpleMeshSceneObject();
		ClothObjectBack->SetShader(Application->ClothShader);
		ClothObjectBack->SetUniform("uColor", CUniform<color3f>(Colors::Yellow));
		ClothObjectBack->SetUniform("uFlipNormals", CUniform<int>(1));
		ClothObjectBack->SetFeatureEnabled(EDrawFeature::CullFront, true);
		ClothObjectBack->SetFeatureEnabled(EDrawFeature::Wireframe, true);
		Application->RenderPass->AddSceneObject(ClothObjectBack);
	}
	ClothObjectBack->SetMesh(ClothMesh);
}

void CClothSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	ParticlesMutex.lock();
	for (int x = 0; x < Rows - 1; ++ x)
	{
		for (int y = 0; y < Columns - 1; ++ y)
		{
			size_t const Start = (y + x * (Columns - 1)) * 4;

			for (size_t i = 0; i < 4; ++ i)
			{
				static vec2i const Offsets[] =
				{
					vec2i(0, 0),
					vec2i(0, 1),
					vec2i(1, 1),
					vec2i(1, 0),
				};

				ClothMesh->Vertices[Start + i].Position = GetParticle(vec2i(x, y) + Offsets[i])->PositionFrames[CurrentFrame];
				ClothMesh->Vertices[Start + i].Normal = vec3f(0, 0, 1);
			}
		}
	}
	ParticlesMutex.unlock();

	ClothObjectFront->SetMesh(ClothMesh);
	ClothObjectBack->SetMesh(ClothMesh);
}

CClothSimulation::SParticle * CClothSimulation::GetParticle(vec2i const & Index)
{
	return Particles[Index.X * Columns + Index.Y];
}
