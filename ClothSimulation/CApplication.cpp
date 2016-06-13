
#include "CApplication.h"
#include "Util.h"


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
	else if (InstanceOf<SMouseEvent>(Event))
	{
		SMouseEvent MouseEvent = As<SMouseEvent>(Event);

		switch (MouseEvent.Type)
		{
		case SMouseEvent::EType::Click:
			if (MouseEvent.Button == SMouseEvent::EButton::Left)
			{
				if (! MouseEvent.Pressed)
				{
					ray3f const Ray = FreeCamera->GetPickingRay(MouseEvent.Location, Window->GetSize());
					RigidDynamicsSimulation->PickObject(Ray);
				}
			}

			break;
		}
	}
}


void CApplication::InitializeEngine()
{
	GraphicsAPI->Init(new Graphics::COpenGLImplementation());
	WindowManager->Init(GraphicsAPI);
	TimeManager->Init(WindowManager);

	Window = WindowManager->CreateWindow(vec2i(2400, 1300), "Physically-Based Animation", EWindowType::Windowed);
	Window->SetPosition(vec2i(50, 50));

	GraphicsContext = GraphicsAPI->GetWindowContext(Window);

	SceneManager->Init(GraphicsAPI);

	AssetManager->Init(GraphicsAPI);
	AssetManager->AddAssetPath("Assets/");
	AssetManager->SetShaderPath("Shaders/");
	AssetManager->SetTexturePath("Textures/");

	RenderTarget = GraphicsContext->GetBackBuffer();
	RenderTarget->SetClearColor(color3f(0.9f));

	GUIManager->Init(Window);
	GUIManager->AddFontFromFile("Assets/GUI/OpenSans.ttf", 18.f);
	Window->AddListener(GUIManager);
	GUIManager->AddListener(this);
}

void CApplication::LoadAssets()
{
	CubeMesh = CGeometryCreator::CreateCube();
	SphereMesh = CGeometryCreator::CreateSphere();

	ClothShader = AssetManager->LoadShader("Cloth");
	GroundShader = AssetManager->LoadShader("Ground");
	DiffuseShader = AssetManager->LoadShader("Diffuse");
	ColorShader = AssetManager->LoadShader("Color");

	GroundTexture = AssetManager->LoadTexture("Ground.png");
	if (GroundTexture)
	{
		GroundTexture->SetMagFilter(ITexture::EFilter::Nearest);
		GroundTexture->SetWrapMode(ITexture::EWrapMode::Clamp);
	}
}

void CApplication::SetupScene()
{
	RenderPass = new CRenderPass(GraphicsContext);
	RenderPass->SetRenderTarget(RenderTarget);
	SceneManager->AddRenderPass(RenderPass);

	FreeCamera = new CPerspectiveCamera(Window->GetAspectRatio());
	FreeCamera->SetPosition(vec3f(0, 1.25f, 3));
	FreeCamera->SetFocalLength(0.4f);
	FreeCamera->SetFarPlane(10000.f);

	CCameraController * Controller = new CGamePadCameraController(FreeCamera);
	Controller->SetTheta(-Constants32::Pi / 2);
	Controller->SetPhi(0);
	Window->AddListener(Controller);
	TimeManager->MakeUpdateTick(0.02)->AddListener(Controller);

	RenderPass->SetActiveCamera(FreeCamera);
}

void CApplication::AddSceneObjects()
{
	GroundObject = new CSimpleMeshSceneObject();
	GroundObject->SetMesh(CubeMesh);
	GroundObject->SetShader(GroundShader);
	GroundObject->SetScale(vec3f(16, 1, 16));
	GroundObject->SetPosition(vec3f(0, 0, 0));
	GroundObject->SetTexture("uTexture", GroundTexture);
	RenderPass->AddSceneObject(GroundObject);
	
	CDirectionalLight * Light = new CDirectionalLight();
	Light->SetDirection(vec3f(1, -2, -2));
	RenderPass->AddLight(Light);

	PointLight = new CPointLight();
	RenderPass->AddLight(PointLight);

	LineObject = new CLineSceneObject();
	LineObject->SetShader(ColorShader);
	RenderPass->AddSceneObject(LineObject);
}


void CApplication::MainLoop()
{
	RigidDynamicsSimulation = new CRigidDynamicsSimulation();
	SimulationSystem->AddSimulation(RigidDynamicsSimulation);
	SimulationSystem->Start(RenderPass);

	TimeManager->Start();
	while (WindowManager->Run())
	{
		TimeManager->Update();

		LineObject->ResetLines();
		LineObject->AddLine(vec3f(0.2f, 1.25f, 0), vec3f(0.45f, 1.25f, 0), Colors::Red);
		LineObject->AddLine(vec3f(0.45f, 1.25f, 0), vec3f(0.7f, 1.25f, 0), Colors::Green);

		if (Window->IsKeyDown(EKey::Space) || Window->IsKeyDown(EKey::Z) || Window->IsKeyDown(EKey::X))
		{
			for (int i = 1; i <= 2; ++ i)
			{
				double const Magnitude = 50.0;
				double const ControlRegion = 0.2;
				vec3d GoalPosition = RigidDynamicsSimulation->Boxes[i]->OriginalTranslation;
				if (Window->IsKeyDown(EKey::Z))
				{
					std::swap(GoalPosition.X, GoalPosition.Z);
				}
				if (Window->IsKeyDown(EKey::X))
				{
					std::swap(GoalPosition.X, GoalPosition.Z);
					GoalPosition.Z *= -1;
				}
				vec3d Movement = GoalPosition - RigidDynamicsSimulation->Boxes[i]->GetTranslation();

				printf("Length %d: %.6f %s\n", i, Movement.Length(), (Movement.Length() < ControlRegion ? "under gravity control" : ""));
				//if (Movement.Length() < ControlRegion)
					Movement += 
				//	//Interpolate(vec3d(), 
						vec3d(0, 9.8 * RigidDynamicsSimulation->Boxes[i]->m / Magnitude, 0)
				//		//, Movement.Length() / ControlRegion)
					;
				RigidDynamicsSimulation->Boxes[i]->AppliedForce = Magnitude * ToEigen(Movement);
			}
		}
		if (Window->IsKeyDown(EKey::O))
		{
			RigidDynamicsSimulation->Boxes[1]->AppliedTorque.x() = 0.01;
		}
		
		// GUI
		GUIManager->NewFrame();
		SimulationSystem->GUI();

		SimulationSystem->Update();
		PointLight->SetPosition(FreeCamera->GetPosition());

		// Draw
		RenderTarget->ClearColorAndDepth();
		SceneManager->DrawAll();
		ImGui::Render();
		Window->SwapBuffers();
	}

	SimulationSystem->Stop();
}
