
#pragma once

#include <ionEngine.h>
#include "CLagrangianSimulation.h"
#include "CParticleSystemSceneObject.h"


class CApplication : public Singleton<CApplication>, public IEventListener
{

public:

	void Run();
	void OnEvent(IEvent & Event);

	SingletonPointer<ion::CWindowManager> WindowManager;
	SingletonPointer<ion::CTimeManager> TimeManager;
	SingletonPointer<ion::CSceneManager> SceneManager;
	SingletonPointer<ion::CAssetManager> AssetManager;
	SingletonPointer<ion::CGUIManager> GUIManager;
	SingletonPointer<ion::CGraphicsAPI> GraphicsAPI;

	ion::CWindow * Window = nullptr;
	SharedPointer<ion::Graphics::IGraphicsContext> GraphicsContext;
	ion::Scene::CRenderPass * RenderPass = nullptr;
	ion::Scene::CRenderPass * PostProcessPassFilter = nullptr;

	static int const NumBlurPasses = 1;
	ion::Scene::CRenderPass * PostProcessPassBlurV[NumBlurPasses] = { nullptr };
	ion::Scene::CRenderPass * PostProcessPassBlurH[NumBlurPasses] = { nullptr };
	ion::Scene::CRenderPass * PostProcessPassBlend = nullptr;

	SharedPointer<ion::Graphics::IShaderProgram> ClothShader;
	SharedPointer<ion::Graphics::IShaderProgram> GroundShader;
	SharedPointer<ion::Graphics::IShaderProgram> DiffuseShader;
	SharedPointer<ion::Graphics::IShaderProgram> MeshShader;
	SharedPointer<ion::Graphics::IShaderProgram> ParticleShader;

	SharedPointer<ion::Graphics::IShaderProgram> FilterShader;
	SharedPointer<ion::Graphics::IShaderProgram> BlurVShader;
	SharedPointer<ion::Graphics::IShaderProgram> BlurHShader;
	SharedPointer<ion::Graphics::IShaderProgram> BlendShader;

	SharedPointer<ion::Graphics::IFrameBuffer> SceneBuffer;
	SharedPointer<ion::Graphics::IFrameBuffer> SwapBuffer1;
	SharedPointer<ion::Graphics::IFrameBuffer> SwapBuffer2;

	SharedPointer<ion::Graphics::ITexture2D> SceneColor;
	SharedPointer<ion::Graphics::ITexture2D> SwapColor1;
	SharedPointer<ion::Graphics::ITexture2D> SwapColor2;

	ion::Scene::CSimpleMesh * CubeMesh = nullptr;
	ion::Scene::CSimpleMesh * SphereMesh = nullptr;
	
	SharedPointer<ion::Graphics::ITexture2D> GroundTexture;
	SharedPointer<ion::Graphics::ITexture2D> FireTexture1;

	CParticleSystemSceneObject * ExplosionSystem = nullptr;

protected:

	void InitializeEngine();
	void LoadAssets();
	void SetupScene();
	void AddSceneObjects();
	void MainLoop();

	SharedPointer<ion::Graphics::IRenderTarget> RenderTarget = nullptr;
	ion::Scene::CPerspectiveCamera * FreeCamera = nullptr;
	ion::Scene::CPerspectiveCamera * PlayerCamera = nullptr;
	ion::Scene::CPointLight * PointLight = nullptr;

	CLagrangianSimulation * Simulation = nullptr;

private:

	friend class Singleton<CApplication>;
	CApplication()
	{}

};
