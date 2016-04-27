
#include "CLagrangianSimulation.h"
#include "CApplication.h"

using namespace ion;
using namespace ion::Scene;


CLagrangianSimulation::CLagrangianSimulation()
{
	SParticle * Particle = new SParticle();
	Particle->Mass = Settings.mass;

	AddSceneObjects();
	UpdateSceneObjects(0);
}

void CLagrangianSimulation::Setup()
{
}

void CLagrangianSimulation::SimulateStep(double const TimeDelta)
{
}

void CLagrangianSimulation::GUI()
{
}

void CLagrangianSimulation::Reset()
{
}

void CLagrangianSimulation::AddSceneObjects()
{
	SingletonPointer<CApplication> Application;

	CSimpleMeshSceneObject * TorusObject = new CSimpleMeshSceneObject();
	TorusObject->SetMesh(Application->TorusMesh);
	TorusObject->SetShader(Application->DiffuseShader);

	Application->RenderPass->AddSceneObject(TorusObject);
}

void CLagrangianSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
}
