
#include "CLineSceneObject.h"


using namespace ion;
using namespace ion::Graphics;
using namespace ion::Scene;


CLineSceneObject::CLineSceneObject()
{

}

CLineSceneObject::~CLineSceneObject()
{

}

void CLineSceneObject::Load(CRenderPass * RenderPass)
{
	SingletonPointer<CGraphicsAPI> GraphicsAPI;

	IndexBuffer = GraphicsAPI->CreateIndexBuffer();
	VertexBuffer = GraphicsAPI->CreateVertexBuffer();
	SInputLayoutElement InputLayout[] =
	{
		{ "vPosition", 3, EAttributeType::Float },
		{ "vColor", 3, EAttributeType::Float },
	};
	VertexBuffer->SetInputLayout(InputLayout, ION_ARRAYSIZE(InputLayout));

	PipelineState = RenderPass->GetGraphicsContext()->CreatePipelineState();
	PipelineState->SetProgram(Shader);
	PipelineState->SetIndexBuffer(IndexBuffer);
	PipelineState->SetVertexBuffer(0, VertexBuffer);
	PipelineState->SetPrimitiveType(EPrimitiveType::Line);

	RenderPass->PreparePipelineStateForRendering(PipelineState, this);
	Loaded[RenderPass] = true;

}

void CLineSceneObject::Draw(CRenderPass * RenderPass)
{
	if (DataNeedsUpload)
	{
		IndexBuffer->UploadData(Indices);
		VertexBuffer->UploadData(Vertices);

		DataNeedsUpload = false;
	}

	RenderPass->SubmitPipelineStateForRendering(PipelineState, this);
}

void CLineSceneObject::SetShader(SharedPointer<Graphics::IShaderProgram> Shader)
{
	this->Shader = Shader;
	TriggerReload();
}

void CLineSceneObject::ResetLines()
{
	DataNeedsUpload = true;
	Vertices.clear();
	Indices.clear();
	IndexCounter = 0;
}

void CLineSceneObject::AddLine(vec3f const & A, vec3f const & B, color3f const & Color)
{
	DataNeedsUpload = true;

	Vertices.push_back(A.X);
	Vertices.push_back(A.Y);
	Vertices.push_back(A.Z);

	Vertices.push_back(Color.Red);
	Vertices.push_back(Color.Green);
	Vertices.push_back(Color.Blue);

	Vertices.push_back(B.X);
	Vertices.push_back(B.Y);
	Vertices.push_back(B.Z);

	Vertices.push_back(Color.Red);
	Vertices.push_back(Color.Green);
	Vertices.push_back(Color.Blue);

	Indices.push_back((uint) IndexCounter++);
	Indices.push_back((uint) IndexCounter++);
}
