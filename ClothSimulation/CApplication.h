
#pragma once

#include <ionEngine.h>
#include "CClothSimulation.h"


class CApplication : public Singleton<CApplication>, public IEventListener
{

public:

	void Run();
	void OnEvent(IEvent & Event);

	SingletonPointer<CWindowManager> WindowManager;
	SingletonPointer<CTimeManager> TimeManager;
	SingletonPointer<ion::Scene::CSceneManager> SceneManager;
	SingletonPointer<ion::CAssetManager> AssetManager;
	SingletonPointer<CGUIManager> GUIManager;

	CWindow * Window = nullptr;
	ion::Graphics::IGraphicsAPI * GraphicsAPI = nullptr;
	SharedPointer<ion::Graphics::IGraphicsContext> GraphicsContext;
	ion::Scene::CRenderPass * RenderPass = nullptr;

	SharedPointer<ion::Graphics::IShaderProgram> ClothShader;
	SharedPointer<ion::Graphics::IShaderProgram> GroundShader;
	SharedPointer<ion::Graphics::IShaderProgram> DiffuseShader;

	ion::Scene::CSimpleMesh * CubeMesh = nullptr;
	
	SharedPointer<ion::Graphics::ITexture> GroundTexture;

	CClothSimulation ClothSimulation;

protected:

	void InitializeEngine();
	void LoadAssets();
	void SetupScene();
	void AddSceneObjects();
	void MainLoop();

	SharedPointer<ion::Graphics::IRenderTarget> RenderTarget = nullptr;
	ion::Scene::CPerspectiveCamera * FreeCamera = nullptr;
	ion::Scene::CPointLight * PointLight = nullptr;

	int DisplayedFrame = 0;
	std::atomic<int> SimulatedFrames = 1;

private:

	friend class Singleton<CApplication>;
	CApplication()
	{}

};
