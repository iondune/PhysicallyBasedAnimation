
#include "CApplication.h"
#include "Util.h"


using namespace ion;
using namespace ion::Scene;
using namespace ion::Graphics;


bool needUpdate = true;

CLineSceneObject * lns = nullptr;

CSimpleMeshSceneObject * jnt1 = nullptr;
CSimpleMeshSceneObject * jnt2 = nullptr;
CSimpleMeshSceneObject * jnt3 = nullptr;
CSimpleMeshSceneObject * jnt1hlf = nullptr;
CSimpleMeshSceneObject * jnt2hlf = nullptr;
CSimpleMeshSceneObject * jnt3hlf = nullptr;
CSimpleMeshSceneObject * gl = nullptr;


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
				ToggleBool(Recording);
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
				ToggleBool(ArmTracking);
				break;

			case EKey::Z:
				for (int i = 1; i <= 2; ++ i)
				{
					RigidDynamicsSimulation->Boxes[i]->GoalTranslation = RigidDynamicsSimulation->Boxes[i]->OriginalTranslation;
					std::swap(RigidDynamicsSimulation->Boxes[i]->GoalTranslation.X, RigidDynamicsSimulation->Boxes[i]->GoalTranslation.Z);
				}
				break;
			case EKey::X:
				for (int i = 1; i <= 2; ++ i)
				{
					RigidDynamicsSimulation->Boxes[i]->GoalTranslation = RigidDynamicsSimulation->Boxes[i]->OriginalTranslation;
					std::swap(RigidDynamicsSimulation->Boxes[i]->GoalTranslation.X, RigidDynamicsSimulation->Boxes[i]->GoalTranslation.Z);
					RigidDynamicsSimulation->Boxes[i]->GoalTranslation.Z *= -1;
				}
				break;
			case EKey::C:
				for (int i = 1; i <= 2; ++ i)
				{
					RigidDynamicsSimulation->Boxes[i]->GoalTranslation = (i == 1 ? jnt1hlf->GetPosition() : jnt2hlf->GetPosition());
				}
				break;
			case EKey::V:
				for (int i = 1; i <= 2; ++ i)
				{
					RigidDynamicsSimulation->Boxes[i]->GoalTranslation = RigidDynamicsSimulation->Boxes[i]->OriginalTranslation;
				}
				break;

			case EKey::KeyPad1:
				GoalPosition = vec3f(0.330f, 0.190f, 0.0f);
				needUpdate = true;
				break;

			case EKey::KeyPad2:
				GoalPosition = vec3f(0.0f, -0.050f, -0.320f);
				needUpdate = true;
				break;

			case EKey::KeyPad3:
				GoalPosition = vec3f(0.230f, -0.050f, -0.320f);
				needUpdate = true;
				break;

			case EKey::KeyPad4:
				GoalPosition = vec3f(0.280f, -0.360f, -0.110f);
				needUpdate = true;
				break;
			}
		}
		else if (KeyboardEvent.Pressed)
		{
			switch (KeyboardEvent.Key)
			{
			case EKey::I:
				GoalPosition.Z -= 0.01f;
				needUpdate = true;
				break;
			case EKey::K:
				GoalPosition.Z += 0.01f;
				needUpdate = true;
				break;

			case EKey::J:
				GoalPosition.X -= 0.01f;
				needUpdate = true;
				break;
			case EKey::L:
				GoalPosition.X += 0.01f;
				needUpdate = true;
				break;

			case EKey::U:
				GoalPosition.Y += 0.01f;
				needUpdate = true;
				break;
			case EKey::O:
				GoalPosition.Y -= 0.01f;
				needUpdate = true;
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
	SphereMesh = CGeometryCreator::CreateSphere(0.5f, 12, 8);

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

	lns = LineObject = new CLineSceneObject();
	LineObject->SetShader(ColorShader);
	RenderPass->AddSceneObject(LineObject);

	jnt1 = new CSimpleMeshSceneObject();
	jnt1->SetMesh(SphereMesh);
	jnt1->SetScale(0.0125f);
	jnt1->SetShader(DiffuseShader);
	jnt1->SetUniform("uColor", CUniform<color3f>(Colors::Magenta));
	jnt1->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(jnt1);

	jnt2 = new CSimpleMeshSceneObject();
	jnt2->SetMesh(SphereMesh);
	jnt2->SetScale(0.0125f);
	jnt2->SetShader(DiffuseShader);
	jnt2->SetUniform("uColor", CUniform<color3f>(Colors::Orange));
	jnt2->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(jnt2);

	jnt3 = new CSimpleMeshSceneObject();
	jnt3->SetMesh(SphereMesh);
	jnt3->SetScale(0.0125f);
	jnt3->SetShader(DiffuseShader);
	jnt3->SetUniform("uColor", CUniform<color3f>(Colors::Cyan));
	jnt3->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(jnt3);

	gl = new CSimpleMeshSceneObject();
	gl->SetMesh(SphereMesh);
	gl->SetScale(0.01f);
	gl->SetShader(DiffuseShader);
	gl->SetUniform("uColor", CUniform<color3f>(Colors::Yellow * 0.75f));
	gl->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(gl);

	jnt1hlf = new CSimpleMeshSceneObject();
	jnt1hlf->SetMesh(CubeMesh);
	jnt1hlf->SetScale(0.0125f);
	jnt1hlf->SetShader(DiffuseShader);
	jnt1hlf->SetUniform("uColor", CUniform<color3f>(Colors::Magenta));
	jnt1hlf->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(jnt1hlf);

	jnt2hlf = new CSimpleMeshSceneObject();
	jnt2hlf->SetMesh(CubeMesh);
	jnt2hlf->SetScale(0.0125f);
	jnt2hlf->SetShader(DiffuseShader);
	jnt2hlf->SetUniform("uColor", CUniform<color3f>(Colors::Orange));
	jnt2hlf->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(jnt2hlf);

	jnt3hlf = new CSimpleMeshSceneObject();
	jnt3hlf->SetMesh(CubeMesh);
	jnt3hlf->SetScale(0.0125f);
	jnt3hlf->SetShader(DiffuseShader);
	jnt3hlf->SetUniform("uColor", CUniform<color3f>(Colors::Cyan));
	jnt3hlf->SetFeatureEnabled(EDrawFeature::Wireframe, true);
	RenderPass->AddSceneObject(jnt3hlf);
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
		//LineObject->AddLine(vec3f(0.2f, 1.25f, 0), vec3f(0.45f, 1.25f, 0), Colors::Red);
		//LineObject->AddLine(vec3f(0.45f, 1.25f, 0), vec3f(0.7f, 1.25f, 0), Colors::Green);

		if (needUpdate)
		{
			DoCCD_IK(GoalPosition);
			gl->SetPosition(vec3f(0.2f, 1.25f, 0) + GoalPosition);
			needUpdate = false;
		}

		vec3f cent = vec3f(0.2f, 1.25f, 0);
		lns->AddLine(cent, jnt1->GetPosition(), Colors::Magenta);
		lns->AddLine(jnt1->GetPosition(), jnt2->GetPosition(), Colors::Orange);
		lns->AddLine(jnt2->GetPosition(), jnt3->GetPosition(), Colors::Cyan);

		if (ArmTracking)
		{
			for (int i = 1; i <= 2; ++ i)
			{
				double const Magnitude = 50.0;
				double const ControlRegion = 0.2;
				vec3d Movement = RigidDynamicsSimulation->Boxes[i]->GoalTranslation - RigidDynamicsSimulation->Boxes[i]->GetTranslation();
				Movement += vec3d(0, 9.8 * RigidDynamicsSimulation->Boxes[i]->m / Magnitude, 0);

				RigidDynamicsSimulation->Boxes[i]->AppliedForce = Magnitude * ToEigen(Movement);
			}
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


		if (Recording)
		{
			CImage * Image = RenderTarget->ReadImage();

			static int FrameCounter = 0;
			string FileName = String::Build("Frames/Frame%06d.png", FrameCounter);
			FrameCounter ++;

			Image->Write(FileName);
		}

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

		glm::mat4 getLocalHalfTransformation()
		{
			glm::mat4 Trans = glm::translate(glm::mat4(1.f), glm::vec3(Length / 2.f, 0, 0));

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

		glm::mat4 getHalfTransformation()
		{
			glm::mat4 Trans = getLocalHalfTransformation();

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

		vec3f const getHalfLocation()
		{
			glm::vec4 v(0, 0, 0, 1);
			v = getHalfTransformation() * v;

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
			for (int t = 0; t < ION_ARRAYSIZE(Joints); ++t)
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
			Delta /= 1.01f;
		}
	}

	vec3f cent = vec3f(0.2f, 1.25f, 0);
	jnt1->SetPosition(cent + Root.getLocation());
	jnt1hlf->SetPosition(cent + Root.getHalfLocation());
	jnt2->SetPosition(cent + Joint1.getLocation());
	jnt2hlf->SetPosition(cent + Joint1.getHalfLocation());
	jnt3->SetPosition(cent + Hand.getLocation());
	jnt3hlf->SetPosition(cent + Hand.getHalfLocation());
}
