
#pragma once

#include <ionEngine.h>
#include "CSimulationSystem.h"
#include "CRigidDynamicsSimulation.h"
#include "CLineSceneObject.h"


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

	SharedPointer<ion::Graphics::IShaderProgram> ClothShader;
	SharedPointer<ion::Graphics::IShaderProgram> GroundShader;
	SharedPointer<ion::Graphics::IShaderProgram> DiffuseShader;
	SharedPointer<ion::Graphics::IShaderProgram> ColorShader;

	ion::Scene::CSimpleMesh * CubeMesh = nullptr;
	ion::Scene::CSimpleMesh * SphereMesh = nullptr;
	
	SharedPointer<ion::Graphics::ITexture> GroundTexture;

	ion::Scene::CSimpleMeshSceneObject * GroundObject = nullptr;
	CLineSceneObject * LineObject = nullptr;

protected:

	void InitializeEngine();
	void LoadAssets();
	void SetupScene();
	void AddSceneObjects();
	void MainLoop();

	SharedPointer<ion::Graphics::IRenderTarget> RenderTarget = nullptr;
	ion::Scene::CPerspectiveCamera * FreeCamera = nullptr;
	ion::Scene::CPointLight * PointLight = nullptr;

	SingletonPointer<CSimulationSystem> SimulationSystem;
	CRigidDynamicsSimulation * RigidDynamicsSimulation = nullptr;

private:

	friend class Singleton<CApplication>;
	CApplication()
	{}

};
