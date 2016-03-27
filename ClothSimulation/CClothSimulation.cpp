
#include "CClothSimulation.h"
#include "CApplication.h"

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


void CClothSimulation::AddSceneObjects(ion::Scene::CRenderPass * RenderPass)
{
	SingletonPointer<CApplication> Application;

	ClothMesh = new CSimpleMesh();

	for (int x = 0; x < 10; ++ x)
	{
		for (int y = 0; y < 10; ++ y)
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

	ClothObjectFront = new CSimpleMeshSceneObject();
	ClothObjectFront->SetMesh(ClothMesh);
	ClothObjectFront->SetShader(Application->ClothShader);
	ClothObjectFront->SetUniform("uColor", CUniform<color3f>(Colors::Red));
	ClothObjectFront->SetUniform("uFlipNormals", CUniform<int>(0));
	ClothObjectFront->SetFeatureEnabled(EDrawFeature::CullBack, true);
	RenderPass->AddSceneObject(ClothObjectFront);

	ClothObjectBack = new CSimpleMeshSceneObject();
	ClothObjectBack->SetMesh(ClothMesh);
	ClothObjectBack->SetShader(Application->ClothShader);
	ClothObjectBack->SetUniform("uColor", CUniform<color3f>(Colors::Yellow));
	ClothObjectBack->SetUniform("uFlipNormals", CUniform<int>(1));
	ClothObjectBack->SetFeatureEnabled(EDrawFeature::CullFront, true);
	RenderPass->AddSceneObject(ClothObjectBack);
}

void CClothSimulation::UpdateSceneObjects()
{
	for (int x = 0; x < 10; ++ x)
	{
		for (int y = 0; y < 10; ++ y)
		{
			size_t const Start = (y + x * 10) * 4;

			for (size_t i = 0; i < 4; ++ i)
			{
				static vec2f Offsets[] =
				{
					vec2f(0, 0),
					vec2f(0, 1),
					vec2f(1, 1),
					vec2f(1, 0),
				};

				vec2f Position = vec2f((float) x, (float) y) + Offsets[i];
				Position -= 5;
				Position /= 10;
				Position.Y += 2;

				ClothMesh->Vertices[Start + i].Position = Position;
				ClothMesh->Vertices[Start + i].Normal = vec3f(0, 0, 1);
			}
		}
	}

	ClothObjectFront->SetMesh(ClothMesh);
	ClothObjectBack->SetMesh(ClothMesh);
}
