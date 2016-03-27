
#include "CSphereSlideSimulation.h"
#include "CApplication.h"

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


void CSphereSlideSimulation::Setup()
{
	Radius = 0.1;
	PositionFrames.push_back(vec3d(0.0, 0.2, 0.0));
}

void CSphereSlideSimulation::SimulateStep(double const TimeDelta)
{
	TotalTime += TimeDelta;

	PositionMutex.lock();
	vec3d x0 = PositionFrames.back();
	double radius = 0.5;
	double a = 2.0*TotalTime;
	vec3d x1 = x0;
	x1.Z = radius * sin(a);

	PositionFrames.push_back(x1);
	PositionMutex.unlock();
}

void CSphereSlideSimulation::AddSceneObjects(ion::Scene::CRenderPass * RenderPass)
{
	SingletonPointer<CApplication> Application;

	SphereObject = new CSimpleMeshSceneObject();
	SphereObject->SetMesh(Application->SphereMesh);
	SphereObject->SetShader(Application->DiffuseShader);
	SphereObject->SetScale(vec3f((float) Radius * 2));
	RenderPass->AddSceneObject(SphereObject);
}

void CSphereSlideSimulation::UpdateSceneObjects(uint const CurrentFrame)
{
	PositionMutex.lock();
	SphereObject->SetPosition(PositionFrames[CurrentFrame]);
	PositionMutex.unlock();
}

vec3d const & CSphereSlideSimulation::GetPosition() const
{
	return PositionFrames.back();
}

double CSphereSlideSimulation::GetRadius() const
{
	return Radius;
}
