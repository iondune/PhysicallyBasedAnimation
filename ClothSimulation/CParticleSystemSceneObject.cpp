
#include "CParticleSystemSceneObject.h"

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;
using namespace ion::Animation;


CParticleSystemSceneObject::CParticleSystemSceneObject()
{
}

void CParticleSystemSceneObject::Load(CRenderPass * RenderPass)
{
	vector<f32> const Vertices
	{
		0.5f,  0.5f,   1, 1,
		0.5f, -0.5f,   1, 0,
		-0.5f, -0.5f,   0, 0,
		-0.5f,  0.5f,   0, 1,
	};

	vector<u32> const Indices
	{
		0, 1, 2,
		0, 2, 3,
	};

	SharedPointer<IIndexBuffer> IndexBuffer = RenderPass->GetGraphicsAPI()->CreateIndexBuffer();
	IndexBuffer->UploadData(Indices);
	SharedPointer<IVertexBuffer> VertexBuffer = RenderPass->GetGraphicsAPI()->CreateVertexBuffer();
	VertexBuffer->UploadData(Vertices);
	SInputLayoutElement InputLayout[] =
	{
		{ "vPosition", 2, EAttributeType::Float },
		{ "vTexCoords", 2, EAttributeType::Float },
	};
	VertexBuffer->SetInputLayout(InputLayout, ION_ARRAYSIZE(InputLayout));

	InstanceBuffer = RenderPass->GetGraphicsAPI()->CreateVertexBuffer();
	InstanceBuffer->SetInstancingEnabled(true);
	SInputLayoutElement InstanceLayout[] =
	{
		{ "vInstanceLocation", 3, EAttributeType::Float },
		{ "vInstanceSize", 1, EAttributeType::Float },
		{ "vInstanceRotation", 1, EAttributeType::Float },
		{ "vInstanceAlpha", 1, EAttributeType::Float },
	};
	InstanceBuffer->SetInputLayout(InstanceLayout, ION_ARRAYSIZE(InstanceLayout));

	PipelineState = RenderPass->GetGraphicsContext()->CreatePipelineState();
	PipelineState->SetProgram(Shader);
	PipelineState->SetIndexBuffer(IndexBuffer);
	PipelineState->SetVertexBuffer(0, VertexBuffer);
	PipelineState->SetVertexBuffer(1, InstanceBuffer);
	PipelineState->SetTexture("uTexture", Texture);
	PipelineState->SetBlendMode(EBlendMode::Additive);
	PipelineState->SetFeatureEnabled(EDrawFeature::DisableDepthWrite, true);

	RenderPass->PreparePipelineStateForRendering(PipelineState, this);
	Loaded[RenderPass] = true;
}

vec3f fixedrand3(vec3i const & i)
{
	static uint const table[] =
	{
		7536, 971, 56113, 33383, 21942,
		51540, 10084, 26259, 26232, 58179,
		2964, 16259, 41571, 5579, 28817,
		25963, 32354, 49605, 16521, 20221,
		28907, 13484, 19780, 30340, 52550,
		64848, 55225, 45402, 37892, 7268,
		50588, 7972, 48330, 32418, 5455,
		62714, 24843, 28885, 9639, 24088,
		14415, 44207, 31727, 44828, 24005,
		6979, 52024, 55444, 37575, 38039,
		21595, 10362, 35156, 13057, 33871,
		2868, 929, 45303, 55104, 33692,
		46744, 37682, 52964, 55892, 2244,
		9360, 49240, 31171, 4921, 12733,
		45965, 27227, 7252, 63045, 8768,
		33095, 13437, 48783, 45894, 12307,
		22081, 43735, 54065, 47338, 32245,
		51933, 50783, 9603, 27412, 9642,
		9282, 16623, 31479, 15997, 54287,
		56635, 43307, 35930, 47939, 52740,
	};
	static uint const tablesize = 100;
	uint lookup = i.X + i.Y * 67 + i.Z * 313;

	uint ran = table[lookup % tablesize];
	vec3f ret = vec3f(
		(float) (ran % 16) / 8.f - 1.f,
		(float) ((ran >> 4) % 16) / 8.f - 1.f,
		(float) ((ran >> 8) % 16) / 8.f - 1.f
	);
	return ret;
}

void CParticleSystemSceneObject::Draw(CRenderPass * RenderPass)
{
	//////////////////////
	// Update Particles //
	//////////////////////

	float const Elapsed = (float) TimeManager->GetElapsedTime();

	EmitAccumulator -= Elapsed;
	DeleteAccumulator -= Elapsed;

	if (EmitAccumulator < 0)
	{
		Particles.reserve(Particles.size() + Settings.EmitCount);
		for (int i = 0; i < Settings.EmitCount; ++ i)
		{
			Particles.push_back(SParticle());
			float const OutwardAngle = frand() * 2 * 3.14159f;
			float const OutwardMagnitude = Random::Between(Settings.OutwardsMin, Settings.OutwardsMax);
			float const UpwardMagnitude = Random::Between(Settings.UpwardsMin, Settings.UpwardsMax);
			Particles.back().Velocity = vec3f(-UpwardMagnitude, Sin(OutwardAngle) * OutwardMagnitude, Cos(OutwardAngle) * OutwardMagnitude);
			Particles.back().Position = vec3f(-0.9f, 0.4f, 0);
			Particles.back().Size = Random::Between(Settings.MinSize, Settings.MaxSize);
			Particles.back().Rotation = frand() * 2 * 3.14159f;
			Particles.back().Life = Particles.back().StartLife = Random::Between(Settings.MinLife, Settings.MaxLife);
		}
		std::sort(Particles.begin(), Particles.end());

		EmitAccumulator = Settings.EmitInterval;
	}

	if (DeleteAccumulator < 0)
	{
		for (auto it = Particles.begin(); it != Particles.end(); ++ it)
		{
			if (it->Life <= 0)
			{
				Particles.erase(it, Particles.end());
				break;
			}
		}
		DeleteAccumulator = Settings.DeleteInterval;
	}

	uint ActiveParticles = 0;
	for (auto & Particle : Particles)
	{
		if (Particle.Life < 0)
		{
			break;
		}

		Particle.Life -= Elapsed;

		if (Particle.Life > 0)
		{
			ActiveParticles ++;
		}

		if (Settings.VectorField)
		{
			vec3f Acceleration;
			Acceleration.Y -= 9.8 * Elapsed;
			Acceleration += fixedrand3(Particle.Position * 8.f);

			Particle.Velocity += Acceleration * Elapsed * 1.f;
			Particle.Size += Elapsed * 0.2f;
		}

		Particle.Position += Elapsed * Particle.Velocity;
		Particle.Rotation += Elapsed * Settings.RotationSpeed * 3.14f;
	}

	////////////////////////
	// Send Particle Data //
	////////////////////////

	vector<float> InstanceData;
	InstanceData.reserve(Particles.size() * 6);
	for (auto const & Particle : Particles)
	{
		InstanceData.push_back(Particle.Position.X);
		InstanceData.push_back(Particle.Position.Y);
		InstanceData.push_back(Particle.Position.Z);
		InstanceData.push_back(Particle.Size);
		InstanceData.push_back(Particle.Rotation);

		float const TimeSinceStart = Particle.StartLife - Particle.Life;
		if (TimeSinceStart > Settings.FadeInTime)
		{
			if (Settings.FadeOutParticles)
			{
				InstanceData.push_back(Particle.Life / Particle.StartLife);
			}
			else
			{
				InstanceData.push_back(1.f);
			}
		}
		else
		{
			InstanceData.push_back(TimeSinceStart / Settings.FadeInTime);
		}
	}
	InstanceBuffer->UploadData(InstanceData);


	RenderPass->SubmitPipelineStateForRendering(PipelineState, this, ActiveParticles, 1);
}

void CParticleSystemSceneObject::MakeExplosion(vec3f const & Position)
{
	int const NumParticles = 300;

	Particles.reserve(Particles.size() + NumParticles);
	for (int i = 0; i < NumParticles; ++ i)
	{
		Particles.push_back(SParticle());
		float const OutwardAngle = frand() * 2 * 3.14159f;
		float const OutwardMagnitude = Random::Between(-2.f, 2.f);
		float const UpwardMagnitude = Random::Between(-2.f, 2.f);
		Particles.back().Velocity = vec3f(UpwardMagnitude, Sin(OutwardAngle) * OutwardMagnitude, Cos(OutwardAngle) * OutwardMagnitude) * 0.1f;
		Particles.back().Position = Position;
		Particles.back().Size = Random::Between(Settings.MinSize, Settings.MaxSize) * 3.f;
		Particles.back().Rotation = frand() * 2 * 3.14159f;
		Particles.back().Life = Particles.back().StartLife = Random::Between(Settings.MinLife, Settings.MaxLife);
	}
	std::sort(Particles.begin(), Particles.end());
}

/*
void CParticleDemo::Setup()
{
	CParticleSystemSceneObject * BaseFireSystem = new CParticleSystemSceneObject();
	BaseFireSystem->Shader = Application->ParticleShader;
	BaseFireSystem->Texture = Application->FireTexture1;
	BaseFireSystem->SetPosition(vec3f(4.f, 5.f, -29.f));
	Application->RenderPass->AddSceneObject(BaseFireSystem);

	CParticleSystemSceneObject * FireSparksSystem = new CParticleSystemSceneObject();
	FireSparksSystem->Shader = Application->ParticleShader;
	FireSparksSystem->Texture = Application->FireTexture2;
	FireSparksSystem->SetPosition(BaseFireSystem->GetPosition() + vec3f(0, 2, 0));
	FireSparksSystem->Settings.MinSize = 0.1f;
	FireSparksSystem->Settings.MaxSize = 0.1f;
	FireSparksSystem->Settings.MinLife = 3.f;
	FireSparksSystem->Settings.MaxLife = 5.f;
	FireSparksSystem->Settings.EmitCount = 20;
	FireSparksSystem->Settings.FadeOutParticles = false;
	Application->RenderPass->AddSceneObject(FireSparksSystem);

	CPointLight * FireLight = new CPointLight();
	FireLight->SetPosition(BaseFireSystem->GetPosition() + vec3f(0, 2, 0));
	FireLight->SetColor(color3i(255, 106, 0));
	FireLight->SetRadius(30.f);
	Application->RenderPass->AddLight(FireLight);

	BaseFireSystem = new CParticleSystemSceneObject();
	BaseFireSystem->Shader = Application->ParticleShader;
	BaseFireSystem->Texture = Application->FireTexture1;
	BaseFireSystem->SetPosition(vec3f(28.f, 5.f, -29.f));
	Application->RenderPass->AddSceneObject(BaseFireSystem);

	FireSparksSystem = new CParticleSystemSceneObject();
	FireSparksSystem->Shader = Application->ParticleShader;
	FireSparksSystem->Texture = Application->FireTexture2;
	FireSparksSystem->SetPosition(BaseFireSystem->GetPosition() + vec3f(0, 2, 0));
	FireSparksSystem->Settings.MinSize = 0.1f;
	FireSparksSystem->Settings.MaxSize = 0.1f;
	FireSparksSystem->Settings.MinLife = 3.f;
	FireSparksSystem->Settings.MaxLife = 5.f;
	FireSparksSystem->Settings.EmitCount = 20;
	FireSparksSystem->Settings.FadeOutParticles = false;
	Application->RenderPass->AddSceneObject(FireSparksSystem);

	FireLight = new CPointLight();
	FireLight->SetPosition(BaseFireSystem->GetPosition() + vec3f(0, 2, 0));
	FireLight->SetColor(color3i(255, 106, 0));
	FireLight->SetRadius(30.f);
	Application->RenderPass->AddLight(FireLight);
}
*/