
#include "CApplication.h"

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


void CApplication::Run()
{
	InitializeEngine();
	LoadAssets();
	SetupScene();
	AddSceneObjects();

	MainLoop();
}

void CApplication::OnEvent(IEvent & Event)
{
	if (InstanceOf<SKeyboardEvent>(Event))
	{
		SKeyboardEvent KeyboardEvent = As<SKeyboardEvent>(Event);

		if (! KeyboardEvent.Pressed)
		{
			switch (KeyboardEvent.Key)
			{
			case EKey::F1:
				break;
			case EKey::F:
				RenderPass->SetActiveCamera(FreeCamera);
				break;
			case EKey::G:
				break;
			case EKey::LeftBracket:
				break;
			case EKey::RightBracket:
				break;
			case EKey::P:
				break;
			case EKey::Space:
				break;
			}
		}
	}
}


void CApplication::InitializeEngine()
{
	WindowManager->Init();
	TimeManager->Init();

	Window = WindowManager->CreateWindow(vec2i(1600, 900), "DemoApplication", EWindowType::Windowed);
	Window->AddChild(this);

	GraphicsAPI = new COpenGLAPI();
	GraphicsContext = GraphicsAPI->GetWindowContext(Window);

	SceneManager->Init(GraphicsAPI);
	AssetManager->Init(GraphicsAPI);
	AssetManager->SetAssetPath("Assets/");
	AssetManager->SetShaderPath("Shaders/");
	AssetManager->SetTexturePath("Textures/");

	RenderTarget = GraphicsContext->GetBackBuffer();
	RenderTarget->SetClearColor(color3f(0.9f));

	GUIManager->Init(Window);
	GUIManager->AddFontFromFile("Assets/GUI/OpenSans.ttf", 18.f);
	Window->AddListener(GUIManager);
}

void CApplication::LoadAssets()
{
	CubeMesh = CGeometryCreator::CreateCube();

	ClothShader = AssetManager->LoadShader("Cloth");
	GroundShader = AssetManager->LoadShader("Ground");
	DiffuseShader = AssetManager->LoadShader("Diffuse");

	GroundTexture = AssetManager->LoadTexture("Ground.png");
	if (GroundTexture)
	{
		GroundTexture->SetMagFilter(ITexture::EFilter::Nearest);
		GroundTexture->SetWrapMode(ITexture::EWrapMode::Clamp);
	}
}

void CApplication::SetupScene()
{
	RenderPass = new CRenderPass(GraphicsAPI, GraphicsContext);
	RenderPass->SetRenderTarget(RenderTarget);
	SceneManager->AddRenderPass(RenderPass);

	FreeCamera = new CPerspectiveCamera(Window->GetAspectRatio());
	FreeCamera->SetPosition(vec3f(-1, 0.3f, 0));
	FreeCamera->SetFocalLength(0.4f);
	FreeCamera->SetFarPlane(10000.f);

	CCameraController * Controller = new CCameraController(FreeCamera);
	Controller->SetTheta(0);
	Controller->SetPhi(0);
	Window->AddListener(Controller);
	TimeManager->MakeUpdateTick(0.02)->AddListener(Controller);

	RenderPass->SetActiveCamera(FreeCamera);
}

void CApplication::AddSceneObjects()
{
	CSimpleMeshSceneObject * GroundObject = new CSimpleMeshSceneObject();
	GroundObject->SetMesh(CubeMesh);
	GroundObject->SetShader(GroundShader);
	GroundObject->SetScale(vec3f(16, 1, 16));
	GroundObject->SetPosition(vec3f(0, -1, 0));
	GroundObject->SetTexture("uTexture", GroundTexture);
	RenderPass->AddSceneObject(GroundObject);
	
	CDirectionalLight * Light = new CDirectionalLight();
	Light->SetDirection(vec3f(1, -2, -2));
	RenderPass->AddLight(Light);

	PointLight = new CPointLight();
	RenderPass->AddLight(PointLight);

	ClothSimulation.Setup();
	ClothSimulation.AddSceneObjects(RenderPass);
	ClothSimulation.UpdateSceneObjects(DisplayedFrame);
}


void CApplication::MainLoop()
{
	double const TimeStep = 1e-2;

	bool Running = true;
	bool Simulating = false;
	bool Paused = false;

	std::thread SimulationThread([this, &Running, &Simulating, TimeStep]()
	{
		while (Running)
		{
			if (Simulating)
			{
				ClothSimulation.SimulateStep(TimeStep);
				SimulatedFrames ++;
			}
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
	});

	double StepAccumulator = 0;

	TimeManager->Init();
	while (WindowManager->Run())
	{
		TimeManager->Update();
		
		// GUI
		GUIManager->NewFrame();
		ImGui::SetNextWindowPos(ImVec2(10, 10));
		ImGui::Begin("Simulation");
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		int MaxFrames = SimulatedFrames - 1;
		ImGui::Separator();
		if (Simulating)
		{
			if (ImGui::Button("Stop"))
			{
				Simulating = false;
			}
		}
		else
		{
			if (ImGui::Button("Simulate"))
			{
				Simulating = true;
			}
		}
		ImGui::SameLine();
		if (Paused)
		{
			if (ImGui::Button("Play"))
			{
				Paused = false;
			}
		}
		else
		{
			if (ImGui::Button("Pause"))
			{
				Paused = true;
			}
		}
		ImGui::Text("Simulated Frames: %d", MaxFrames);
		if (ImGui::SliderInt("Current Frame", &DisplayedFrame, 0, MaxFrames))
		{
			ClothSimulation.UpdateSceneObjects(DisplayedFrame);
			Paused = true;
		}
		if (ImGui::Button("<< Previous"))
		{
			if (DisplayedFrame > 0)
			{
				ClothSimulation.UpdateSceneObjects(DisplayedFrame--);
			}
			Paused = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Next >>"))
		{
			if (DisplayedFrame < MaxFrames)
			{
				ClothSimulation.UpdateSceneObjects(DisplayedFrame++);
			}
			Paused = true;
		}
		ImGui::End();

		if (! Paused)
		{
			StepAccumulator += TimeManager->GetElapsedTime();

			if (StepAccumulator > TimeStep)
			{
				StepAccumulator = 0;

				if (DisplayedFrame < MaxFrames)
				{
					ClothSimulation.UpdateSceneObjects(DisplayedFrame++);
				}
			}
		}

		PointLight->SetPosition(FreeCamera->GetPosition());

		// Draw
		RenderTarget->ClearColorAndDepth();
		SceneManager->DrawAll();
		ImGui::Render();
		Window->SwapBuffers();
	}

	Running = false;
	SimulationThread.join();
}
