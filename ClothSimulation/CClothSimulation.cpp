
#include "CClothSimulation.h"

using namespace ion;
using namespace ion::Scene;


void CClothSimulation::AddSceneObjects(ion::Scene::CRenderPass * RenderPass)
{
	CSimpleMesh * ClothMesh = new CSimpleMesh();

	for (int x = 0; x < 10; ++ x)
	{
		for (int y = 0; y < 10; ++ y)
		{
			vec2f Position(x, y);
			Position -= 5;
			Position /= 10;
			Position += 2;

			CSimpleMesh::SVertex Vertex;
			Vertex.Position = Position;
			ClothMesh->Vertices.push_back(Vertex);


		}
	}
}
