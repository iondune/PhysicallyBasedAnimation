
#include "CLagrangianSimulation.h"
#include "CApplication.h"

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


CLagrangianSimulation::CLagrangianSimulation()
{
	SParticle * Particle = new SParticle();
	Particle->Mass = Settings.mass;
	Particle->PositionFrames.push_back(0.0);
	Particle->VelocityFrames.push_back(0.0);
	Particles.push_back(Particle);

	AddSceneObjects();
	UpdateSceneObjects(0);
}

void CLagrangianSimulation::Setup()
{
}

void CLagrangianSimulation::SimulateStep(double const TimeDelta)
{

	ParticlesMutex.lock();
	for (SParticle * particle : Particles)
	{
		particle->VelocityFrames.push_back(particle->VelocityFrames.back() + TimeDelta * 0.01);
		particle->PositionFrames.push_back(particle->PositionFrames.back() + TimeDelta * particle->VelocityFrames.back());
	}
	ParticlesMutex.unlock();
}

void CLagrangianSimulation::GUI()
{
}

void CLagrangianSimulation::Reset()
{
	ParticlesMutex.lock();
	for (SParticle * particle : Particles)
	{
		particle->VelocityFrames.resize(1);
		particle->PositionFrames.resize(1);
	}
	ParticlesMutex.unlock();
}

void CLagrangianSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	BoundaryMesh = new CSimpleMeshSceneObject();
	BoundaryMesh->SetMesh(CGeometryCreator::CreateTorus((float) RingRadius, (float) TubeRadius, 20, 10));
	BoundaryMesh->SetShader(Application->DiffuseShader);
	BoundaryMesh->SetUniform("uColor", CUniform<color3f>(Colors::Cyan));
	BoundaryMesh->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	Application->RenderPass->AddSceneObject(BoundaryMesh);

	for (auto Particle : Particles)
	{
		Particle->DebugObject = new CSimpleMeshSceneObject();
		Particle->DebugObject->SetMesh(Application->SphereMesh);
		Particle->DebugObject->SetScale(0.02f);
		Particle->DebugObject->SetShader(Application->DiffuseShader);
		Particle->DebugObject->SetUniform("uColor", CUniform<color3f>(Colors::Red));
		Application->RenderPass->AddSceneObject(Particle->DebugObject);
	}
}

void CLagrangianSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	VisibleFrame = CurrentFrame;

	ParticlesMutex.lock();
	for (auto Particle : Particles)
	{
		Particle->DebugObject->SetPosition(QToCartesian(Particle->PositionFrames[CurrentFrame]));
	}
	ParticlesMutex.unlock();
}

void CLagrangianSimulation::PickParticle(ray3f const & Ray)
{
}

vec3f CLagrangianSimulation::QToCartesian(vec2f const & Angles)
{
	return vec3f(
		(float) ((RingRadius + TubeRadius * Cos(Angles.X)) * Cos(Angles.Y)),
		(float) (TubeRadius * Sin(Angles.X)),
		(float) ((RingRadius + TubeRadius * Cos(Angles.X)) * Sin(Angles.Y))
	);
}
