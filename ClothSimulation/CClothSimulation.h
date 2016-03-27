
#pragma once

#include <ionEngine.h>


class CClothSimulation
{

public:

	void AddSceneObjects(ion::Scene::CRenderPass * RenderPass);
	void UpdateSceneObjects();

protected:

	ion::Scene::CSimpleMesh * ClothMesh = nullptr;
	ion::Scene::CSimpleMeshSceneObject * ClothObjectFront = nullptr;
	ion::Scene::CSimpleMeshSceneObject * ClothObjectBack = nullptr;

};
