
#include "CApplication.h"

using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;
using namespace ion::Animation;


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
				RenderPass->SetActiveCamera(PlayerCamera);
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
					//ClothSimulation->PickParticle(Ray);
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

	Window = WindowManager->CreateWindow(vec2i(1600, 900), "DemoApplication", EWindowType::Windowed);

	GraphicsContext = GraphicsAPI->GetWindowContext(Window);

	SceneManager->Init(GraphicsAPI);

	AssetManager->Init(GraphicsAPI);
	AssetManager->AddAssetPath("Assets/");
	AssetManager->SetShaderPath("Shaders/");
	AssetManager->SetTexturePath("Textures/");

	RenderTarget = GraphicsContext->GetBackBuffer();

	SceneBuffer = GraphicsContext->CreateFrameBuffer();
	SceneBuffer->SetClearColor(color3f(0.05f));

	SceneColor = GraphicsAPI->CreateTexture2D(Window->GetSize(), ITexture::EMipMaps::False, ITexture::EFormatComponents::RGB, ITexture::EInternalFormatType::Fix8);
	SceneColor->SetMinFilter(ITexture::EFilter::Nearest);
	SceneColor->SetMagFilter(ITexture::EFilter::Nearest);
	SceneColor->SetWrapMode(ITexture::EWrapMode::Clamp);
	SharedPointer<IDepthBuffer> SceneDepth = GraphicsAPI->CreateDepthBuffer(Window->GetSize());
	SceneBuffer->AttachColorTexture(SceneColor, 0);
	SceneBuffer->AttachDepthBuffer(SceneDepth);
	if (! SceneBuffer->CheckCorrectness())
	{
		Log::Error("Frame buffer scene not valid!");
	}

	SwapBuffer1 = GraphicsContext->CreateFrameBuffer();

	SwapColor1 = GraphicsAPI->CreateTexture2D(Window->GetSize(), ITexture::EMipMaps::False, ITexture::EFormatComponents::RGB, ITexture::EInternalFormatType::Fix8);
	SwapColor1->SetMinFilter(ITexture::EFilter::Nearest);
	SwapColor1->SetMagFilter(ITexture::EFilter::Nearest);
	SwapColor1->SetWrapMode(ITexture::EWrapMode::Clamp);
	SharedPointer<IDepthBuffer> SwapDepth1 = GraphicsAPI->CreateDepthBuffer(Window->GetSize());
	SwapBuffer1->AttachColorTexture(SwapColor1, 0);
	SwapBuffer1->AttachDepthBuffer(SwapDepth1);
	if (! SwapBuffer1->CheckCorrectness())
	{
		Log::Error("Frame buffer swap 1 not valid!");
	}

	SwapBuffer2 = GraphicsContext->CreateFrameBuffer();

	SwapColor2 = GraphicsAPI->CreateTexture2D(Window->GetSize(), ITexture::EMipMaps::False, ITexture::EFormatComponents::RGB, ITexture::EInternalFormatType::Fix8);
	SwapColor2->SetMinFilter(ITexture::EFilter::Nearest);
	SwapColor2->SetMagFilter(ITexture::EFilter::Nearest);
	SwapColor2->SetWrapMode(ITexture::EWrapMode::Clamp);
	SharedPointer<IDepthBuffer> SwapDepth2 = GraphicsAPI->CreateDepthBuffer(Window->GetSize());
	SwapBuffer2->AttachColorTexture(SwapColor2, 0);
	SwapBuffer2->AttachDepthBuffer(SwapDepth2);
	if (! SwapBuffer2->CheckCorrectness())
	{
		Log::Error("Frame buffer swap 2 not valid!");
	}

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
	MeshShader = AssetManager->LoadShader("Mesh");
	ParticleShader = AssetManager->LoadShader("Particle");

	FilterShader = AssetManager->LoadShader("PPFilter");
	BlurVShader = AssetManager->LoadShader("PPBlurV");
	BlurHShader = AssetManager->LoadShader("PPBlurH");
	BlendShader = AssetManager->LoadShader("PPBlend");

	GroundTexture = AssetManager->LoadTexture("GroundDark.png");
	if (GroundTexture)
	{
		GroundTexture->SetMagFilter(ITexture::EFilter::Nearest);
		GroundTexture->SetWrapMode(ITexture::EWrapMode::Clamp);
	}

	FireTexture1 = AssetManager->LoadTexture("FireParticle1.png");
}

void CApplication::SetupScene()
{
	RenderPass = new CRenderPass(GraphicsContext);
	RenderPass->SetRenderTarget(SceneBuffer);
	SceneManager->AddRenderPass(RenderPass);

	PostProcessPassFilter = new CRenderPass(GraphicsContext);
	PostProcessPassFilter->SetRenderTarget(SwapBuffer1);
	SceneManager->AddRenderPass(PostProcessPassFilter);

	for (int i = 0; i < NumBlurPasses; ++ i)
	{
		PostProcessPassBlurV[i] = new CRenderPass(GraphicsContext);
		PostProcessPassBlurV[i]->SetRenderTarget(SwapBuffer2);
		SceneManager->AddRenderPass(PostProcessPassBlurV[i]);

		PostProcessPassBlurH[i] = new CRenderPass(GraphicsContext);
		PostProcessPassBlurH[i]->SetRenderTarget(SwapBuffer1);
		SceneManager->AddRenderPass(PostProcessPassBlurH[i]);
	}

	PostProcessPassBlend = new CRenderPass(GraphicsContext);
	PostProcessPassBlend->SetRenderTarget(RenderTarget);
	SceneManager->AddRenderPass(PostProcessPassBlend);

	FreeCamera = new CPerspectiveCamera(Window->GetAspectRatio());
	FreeCamera->SetPosition(vec3f(0, 0.25f, 1));
	FreeCamera->SetFocalLength(0.4f);
	FreeCamera->SetFarPlane(10000.f);

	PlayerCamera = new CPerspectiveCamera(Window->GetAspectRatio());
	PlayerCamera->SetFocalLength(0.4f);
	PlayerCamera->SetNearPlane(0.01f);
	PlayerCamera->SetFarPlane(100.f);

	CCameraController * Controller = new CCameraController(FreeCamera);
	Controller->SetTheta(-Constants32::Pi / 2);
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
	GroundObject->SetPosition(vec3f(0, -6, 0));
	GroundObject->SetTexture("uTexture", GroundTexture);
	RenderPass->AddSceneObject(GroundObject);
	
	CDirectionalLight * Light = new CDirectionalLight();
	Light->SetDirection(vec3f(1, -2, -2));
	RenderPass->AddLight(Light);

	PointLight = new CPointLight();
	RenderPass->AddLight(PointLight);


	CSimpleMeshSceneObject * PostProcessObjectFilter = new CSimpleMeshSceneObject();
	PostProcessObjectFilter->SetMesh(CGeometryCreator::CreateScreenTriangle());
	PostProcessObjectFilter->SetShader(FilterShader);
	PostProcessObjectFilter->SetTexture("uTexture", SceneColor);
	PostProcessObjectFilter->SetFeatureEnabled(EDrawFeature::DisableDepthTest, true);
	PostProcessObjectFilter->SetFeatureEnabled(EDrawFeature::DisableDepthWrite, true);
	PostProcessPassFilter->AddSceneObject(PostProcessObjectFilter);

	for (int i = 0; i < NumBlurPasses; ++ i)
	{
		CSimpleMeshSceneObject * PostProcessObjectBlurV = new CSimpleMeshSceneObject();
		PostProcessObjectBlurV->SetMesh(CGeometryCreator::CreateScreenTriangle());
		PostProcessObjectBlurV->SetShader(BlurVShader);
		PostProcessObjectBlurV->SetTexture("uTexture", SwapColor1);
		PostProcessObjectBlurV->SetFeatureEnabled(EDrawFeature::DisableDepthTest, true);
		PostProcessObjectBlurV->SetFeatureEnabled(EDrawFeature::DisableDepthWrite, true);
		PostProcessPassBlurV[i]->AddSceneObject(PostProcessObjectBlurV);

		CSimpleMeshSceneObject * PostProcessObjectBlurH = new CSimpleMeshSceneObject();
		PostProcessObjectBlurH->SetMesh(CGeometryCreator::CreateScreenTriangle());
		PostProcessObjectBlurH->SetShader(BlurVShader);
		PostProcessObjectBlurH->SetTexture("uTexture", SwapColor2);
		PostProcessObjectBlurH->SetFeatureEnabled(EDrawFeature::DisableDepthTest, true);
		PostProcessObjectBlurH->SetFeatureEnabled(EDrawFeature::DisableDepthWrite, true);
		PostProcessPassBlurH[i]->AddSceneObject(PostProcessObjectBlurH);
	}

	CSimpleMeshSceneObject * PostProcessObjectBlend = new CSimpleMeshSceneObject();
	PostProcessObjectBlend->SetMesh(CGeometryCreator::CreateScreenTriangle());
	PostProcessObjectBlend->SetShader(BlendShader);
	PostProcessObjectBlend->SetTexture("uScene", SceneColor);
	PostProcessObjectBlend->SetTexture("uHighPass", SwapColor1);
	PostProcessObjectBlend->SetFeatureEnabled(EDrawFeature::DisableDepthTest, true);
	PostProcessObjectBlend->SetFeatureEnabled(EDrawFeature::DisableDepthWrite, true);
	PostProcessPassBlend->AddSceneObject(PostProcessObjectBlend);
}


void CApplication::MainLoop()
{
	Simulation = new CLagrangianSimulation();
	Simulation->PlayerMesh = AssetManager->LoadMesh("SpaceShip.obj");
	Simulation->PlayerMesh->CalculateNormalsPerFace();
	Simulation->PlayerMesh->Material.LoadTextures();
	Simulation->AddSceneObjects();
	Simulation->UpdateSceneObjects();

	double Accumulator = 0;

	TimeManager->Start();
	while (WindowManager->Run())
	{
		TimeManager->Update();

		Accumulator += TimeManager->GetElapsedTime();
		float const Elapsed = (float) TimeManager->GetElapsedTime();


		double const TimeStep = 0.01;
		if (Accumulator > TimeStep)
		{
			Accumulator -= TimeStep;

			if (Accumulator > TimeStep * 2)
				Accumulator = TimeStep * 2;

			Simulation->SimulateStep(TimeStep);

			vec3f const PlayerPosition = Simulation->QToCartesian(Simulation->Player->Position);
			vec3f const TowardsCenter = (Simulation->ClosestCenter(Simulation->Player->Position) - PlayerPosition).GetNormalized();
			vec3f const Forward = vec3f(1, 0, 0)
				.RotateAround(vec3f(0, 1, 0), (float) Simulation->Player->Heading)
				.RotateAround(vec3f(0, 0, 1), Constants32::Pi / 2 - (float) Simulation->Player->Position.X)
				.RotateAround(vec3f(0, 1, 0), (float) -Simulation->Player->Position.Y);

			vec3f PhysicsForward = Forward;
			std::swap(PhysicsForward.Y, PhysicsForward.Z);

			if (Window->IsKeyDown(EKey::Space))
			{
				Simulation->Player->EngineForce = vec3d(PhysicsForward) * 0.3;
			}
			else
			{
				Simulation->Player->EngineForce = vec3d(0, 0, 0);
			}

			if (Window->IsKeyDown(EKey::A))
			{
				Simulation->Player->Heading += (float) TimeStep * 1.f;
			}
			if (Window->IsKeyDown(EKey::D))
			{
				Simulation->Player->Heading -= (float) TimeStep * 1.f;
			}

			vec3f const GoalPosition = -Forward * 0.2f + TowardsCenter * 0.16f * (-(float) Cos(Simulation->Player->Position.X) * 0.5f + 0.5f);
			vec3f const GoalLookDirection = (PlayerPosition + TowardsCenter * 0.1f) - (GoalPosition + PlayerPosition);

			float const CameraSpringTension = 640.0f;

			PlayerCamera->SetLookDirection(
				//GoalLookDirection);
				Move::Cubic(PlayerCamera->GetLookDirecton(), GoalLookDirection, (float) TimeStep, CameraSpringTension, 0.00005f));
			PlayerCamera->SetPosition(PlayerPosition +
				//GoalPosition);
				Move::Cubic(PlayerCamera->GetPosition() - PlayerPosition, GoalPosition, (float) TimeStep, CameraSpringTension, 0.00005f));
			PlayerCamera->SetUpVector(TowardsCenter);

			Simulation->UpdateSceneObjects();
		}


		// GUI
		GUIManager->NewFrame();

		ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiSetCond_Once);
		if (ImGui::Begin("Settings"))
		{
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Text("Camera position: %.3f %.3f %.3f", FreeCamera->GetPosition().X, FreeCamera->GetPosition().Y, FreeCamera->GetPosition().Z);

			ImGui::Separator();

			ImGui::Text("Player position %.3f %.3f", Simulation->Player->Position.X, Simulation->Player->Position.Y);
			ImGui::Text("Player velocity %.3f %.3f", Simulation->Player->Velocity.X, Simulation->Player->Velocity.Y);
			ImGui::Text("Player interior %.3f", Cos(Simulation->Player->Position.X));

			ImGui::End();
		}
		PointLight->SetPosition(FreeCamera->GetPosition());

		// Draw
		RenderTarget->ClearColorAndDepth();
		SceneBuffer->ClearColorAndDepth();
		SwapBuffer1->ClearColorAndDepth();
		SwapBuffer2->ClearColorAndDepth();

		SceneManager->DrawAll();
		ImGui::Render();
		Window->SwapBuffers();
	}
}
