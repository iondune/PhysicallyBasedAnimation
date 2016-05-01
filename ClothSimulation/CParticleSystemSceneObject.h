
#pragma once

#include <ionEngine.h>


class CParticleSystemSceneObject : public ion::Scene::ISceneObject
{

public:

	struct SParticle
	{
		float Size = 1;
		float Rotation = 0;
		vec3f Position;
		vec3f Velocity;
		float Life = 0;
		float StartLife = 0;

		bool operator < (SParticle const & Other)
		{
			return Life > Other.Life;
		}
	};

	struct SSettings
	{
		float EmitInterval = 0.15f;
		int EmitCount = 7;
		float UpwardsMin = 0.5f;
		float UpwardsMax = 3.5f;
		float OutwardsMin = 0.f;
		float OutwardsMax = 0.4f;
		float MinSize = 0.8f * 0.006f;
		float MaxSize = 3.2f * 0.006f;
		float MinLife = 1.f;
		float MaxLife = 2.f;
		float DeleteInterval = 0.2f;
		float RotationSpeed = 0.5f;
		float FadeInTime = 0.2f;
		bool FadeOutParticles = true;
		bool VectorField = false;
	};

	CParticleSystemSceneObject();

	void Load(ion::Scene::CRenderPass * RenderPass);
	void Draw(ion::Scene::CRenderPass * RenderPass);

	void MakeExplosion(vec3f const & Position);

	SingletonPointer<ion::CTimeManager> TimeManager;

	SharedPointer<ion::Graphics::IPipelineState> PipelineState;
	SharedPointer<ion::Graphics::IShaderProgram> Shader;
	SharedPointer<ion::Graphics::ITexture2D> Texture;
	SharedPointer<ion::Graphics::IVertexBuffer> InstanceBuffer;

	SSettings Settings;
	vector<SParticle> Particles;
	float EmitAccumulator = 0;
	float DeleteAccumulator = 0;

};
