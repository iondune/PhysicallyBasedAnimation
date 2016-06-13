
#pragma once

#include <ionScene.h>


class CLineSceneObject : public ion::Scene::ISceneObject
{

public:

	CLineSceneObject();
	~CLineSceneObject();

	virtual void Load(ion::Scene::CRenderPass * RenderPass);
	virtual void Draw(ion::Scene::CRenderPass * RenderPass);

	virtual void SetShader(SharedPointer<ion::Graphics::IShaderProgram> Shader);

	void ResetLines();
	void AddLine(vec3f const & A, vec3f const & B, color3f const & Color);

protected:

	vector<u32> Indices;
	vector<f32> Vertices;
	bool DataNeedsUpload = true;

	size_t IndexCounter = 0;

	SharedPointer<ion::Graphics::IPipelineState> PipelineState;
	SharedPointer<ion::Graphics::IShaderProgram> Shader;

	SharedPointer<ion::Graphics::IIndexBuffer> IndexBuffer;
	SharedPointer<ion::Graphics::IVertexBuffer> VertexBuffer;

};
