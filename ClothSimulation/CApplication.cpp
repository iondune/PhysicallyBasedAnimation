
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

CLineSceneObject * lns = nullptr;

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

	lns = LineObject = new CLineSceneObject();
	LineObject->SetShader(ColorShader);
	RenderPass->AddSceneObject(LineObject);
}

void DoCCD_IK(vec3f const & Goal);

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

		DoCCD_IK(vec3f(0.45f, 0, 0));

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

void DoCCD_IK(vec3f const & Goal)
{
	struct Joint
	{
		Joint * Parent;
		vec3f Rotation = vec3f(0, 3.1415f, 0);
		f32 Length;

		Joint()
			: Parent(0), Length(0)
		{}

		glm::mat4 getLocalTransformation()
		{
			glm::mat4 Trans = glm::translate(glm::mat4(1.f), glm::vec3(Length, 0, 0));

			glm::mat4 Rot = glm::mat4(1.f);
			Rot = glm::rotate(Rot, Rotation.Z, glm::vec3(0, 0, 1));
			Rot = glm::rotate(Rot, Rotation.Y, glm::vec3(0, 1, 0));
			Rot = glm::rotate(Rot, Rotation.X, glm::vec3(1, 0, 0));

			return Rot * Trans;
		}

		glm::mat4 getTransformation()
		{
			glm::mat4 Trans = getLocalTransformation();

			if (Parent)
				Trans = Parent->getTransformation() * Trans;

			return Trans;
		}

		vec3f const getLocation()
		{
			glm::vec4 v(0, 0, 0, 1);
			v = getTransformation() * v;

			return vec3f(v.x, v.y, v.z);
		}
	};

	Joint Root, Joint1, Hand;
	Root.Length = Joint1.Length = 0.25f;
	Joint1.Parent = & Root;
	Hand.Parent = & Joint1;

	Joint * Joints[] = { & Root, & Joint1 };

	vec3f EndEffector(Goal.X, Goal.Y, Goal.Z);

	{
		auto GetValue = [&]() -> f32
		{
			vec3f const HandLoc = Hand.getLocation();
			return Sq(EndEffector.GetDistanceFrom(HandLoc));
		};

		f32 Delta = DegToRad(30.f);
		for (int i = 0; i < 500; ++ i)
		{
			for (int t = 0; t < ION_ARRAYSIZE(Joints); ++ t)
			{
				for (int u = 0; u < 3; ++ u)
				{
					f32 const LastValue = GetValue();
					Joints[t]->Rotation[u] += Delta;
					f32 const AddValue = GetValue();
					Joints[t]->Rotation[u] -= 2 * Delta;
					f32 const SubValue = GetValue();
					Joints[t]->Rotation[u] += Delta;

					if (LastValue < AddValue && LastValue < SubValue)
					{
					}
					else if (AddValue < SubValue)
					{
						Joints[t]->Rotation[u] += Delta;
					}
					else if (SubValue < AddValue)
					{
						Joints[t]->Rotation[u] -= Delta;
					}
					else
					{
					}
				}
			}
			Delta /= 1.25f;
		}
	}

	vec3f cent = vec3f(0.2f, 1.25f, 0);
	lns->AddLine(cent, cent + Root.getLocation(), Colors::Magenta);
	lns->AddLine(cent + Root.getLocation(), cent + Joint1.getLocation(), Colors::Orange);
	lns->AddLine(cent + Joint1.getLocation(), cent + Hand.getLocation(), Colors::Cyan);
}
