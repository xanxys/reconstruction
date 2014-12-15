#include "LoaderPluginPrivatePCH.h"
#include "LoaderPlugin.h"

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <locale>

#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Editor/UnrealEd/Public/AssetSelection.h"
#include "Developer/AssetTools/Public/AssetToolsModule.h"
#include "Developer/DesktopPlatform/Public/DesktopPlatformModule.h"
#include "Developer/RawMesh/Public/RawMesh.h"
#include "ModuleManager.h"
#include "picojson.h"
#include "Runtime/CoreUObject/Public/UObject/UObjectGlobals.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Editor/MainFrame/Public/Interfaces/IMainFrameModule.h"
//#include "Editor/UnrealEd/Private/GeomFitUtils.h"
#include "Slate.h"

#include "LoaderPluginCommands.h"


std::wstring widen(const std::string& s) {
	const int n_wstring = MultiByteToWideChar(CP_UTF8, 0, s.data(), s.size(), nullptr, 0);
	if (n_wstring == 0) {
		throw std::runtime_error("MultiByteToWideChar failed");
	}

	std::vector<wchar_t> buffer(n_wstring);
	const int n_written = MultiByteToWideChar(CP_UTF8, 0, s.data(), s.size(), buffer.data(), buffer.size());
	return std::wstring(buffer.begin(), buffer.end());
}


void FLoaderPlugin::StartupModule()
{
	// This code will execute after your module is loaded into memory (but after global variables are initialized, of course.)
	UE_LOG(LoaderPlugin, Log, TEXT("EqExperiment asset loader plugin initializing"));

	LoaderPluginCommands::Register();
	if (!FModuleManager::Get().IsModuleLoaded("LevelEditor")) {
		// don't try to modify UI when editor is not available (e.g. when cooking project)
		UE_LOG(LoaderPlugin, Log, TEXT("Not adding commands since LevelEditor is not loaded"));
		return;
	}

	commands = MakeShareable(new FUICommandList());
	commands->MapAction(
		LoaderPluginCommands::Get().loadButton,
		FExecuteAction::CreateRaw(this, &FLoaderPlugin::OnLoadButtonClicked),
		FCanExecuteAction());


	FLevelEditorModule& LevelEditorModule = FModuleManager::Get().LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	TSharedPtr<FExtender> Extenders = LevelEditorModule.GetToolBarExtensibilityManager()->GetAllExtenders();

	TSharedPtr<FExtender> MyExtender = MakeShareable(new FExtender);
	MyExtender->AddToolBarExtension("Settings", EExtensionHook::After, commands,
		FToolBarExtensionDelegate::CreateRaw(this, &FLoaderPlugin::AddToolbarExtension));

	FModuleManager::Get().LoadModuleChecked<FDesktopPlatformModule>("DesktopPlatform");

	LevelEditorModule.GetToolBarExtensibilityManager()->AddExtender(MyExtender);
}


// Copied from /Engine/Source/Editor/UnrealEd/Private/GeomFitUtils.cpp
void RefreshCollisionChange(const UStaticMesh * StaticMesh)
{
	for (FObjectIterator Iter(UStaticMeshComponent::StaticClass()); Iter; ++Iter)
	{
		UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(*Iter);
		if (StaticMeshComponent->StaticMesh == StaticMesh)
		{
			// it needs to recreate IF it already has been created
			if (StaticMeshComponent->IsPhysicsStateCreated())
			{
				StaticMeshComponent->RecreatePhysicsState();
			}
		}
	}

	//FEditorSupportDelegates::RedrawAllViewports.Broadcast();
}

static void CalcBoundingBox(const FRawMesh& RawMesh, FVector& Center, FVector& Extents, FVector& LimitVec)
{
	FBox Box(0);

	for (uint32 i = 0; i < (uint32)RawMesh.VertexPositions.Num(); i++)
	{
		Box += RawMesh.VertexPositions[i] * LimitVec;
	}

	Box.GetCenterAndExtents(Center, Extents);
}



int32 GenerateBoxAsSimpleCollision(UStaticMesh* StaticMesh)
{
	/*
	if (!PromptToRemoveExistingCollision(StaticMesh))
	{
		return INDEX_NONE;
	}
	*/

	UBodySetup* bs = StaticMesh->BodySetup;

	// Calculate bounding Box.
	FRawMesh RawMesh;
	FStaticMeshSourceModel& SrcModel = StaticMesh->SourceModels[0];
	SrcModel.RawMeshBulkData->LoadRawMesh(RawMesh);

	FVector unitVec = bs->BuildScale3D;
	FVector Center, Extents;
	CalcBoundingBox(RawMesh, Center, Extents, unitVec);

	bs->Modify();

	// Create new GUID
	bs->InvalidatePhysicsData();

	FKBoxElem BoxElem;
	BoxElem.Center = Center;
	BoxElem.X = Extents.X * 2.0f;
	BoxElem.Y = Extents.Y * 2.0f;
	BoxElem.Z = Extents.Z * 2.0f;
	bs->AggGeom.BoxElems.Add(BoxElem);

	// refresh collision change back to staticmesh components
	RefreshCollisionChange(StaticMesh);

	// Mark staticmesh as dirty, to help make sure it gets saved.
	StaticMesh->MarkPackageDirty();

	return bs->AggGeom.BoxElems.Num() - 1;
}

// Very helpful answer
// https://forums.unrealengine.com/showthread.php?22023-UE4-Automatic-Level-Builder-Construction-and-Pipeline-Scripts&p=103983&viewfull=1#post103983
void FLoaderPlugin::OnLoadButtonClicked() {
#define LOCTEXT_NAMESPACE "LevelEditorToolBar"
	UE_LOG(LoaderPlugin, Log, TEXT("Clicked"));
	
	// Ask input directory. OpenDirectoryDialog cannot be used because it can't expand NAS folder.
	IDesktopPlatform* desktop = FDesktopPlatformModule::Get();
	if (!desktop) {
		UE_LOG(LoaderPlugin, Error, TEXT("Failed to get IDesktopPlatform"));
		return;
	}
	void* ParentWindowWindowHandle = nullptr;
	IMainFrameModule& MainFrameModule = FModuleManager::LoadModuleChecked<IMainFrameModule>(TEXT("MainFrame"));
	const TSharedPtr<SWindow>& MainFrameParentWindow = MainFrameModule.GetParentWindow();
	if (MainFrameParentWindow.IsValid() && MainFrameParentWindow->GetNativeWindow().IsValid()) {
		ParentWindowWindowHandle = MainFrameParentWindow->GetNativeWindow()->GetOSWindowHandle();
	}
	TArray<FString> paths;
	// crash when ParentWindowWindowHandle is nullptr?
	const bool selected = desktop->OpenFileDialog(
		ParentWindowWindowHandle,
		TEXT("Choose Experiment Package"),
		TEXT(""),
		TEXT(""),
		TEXT("Experiment Metadata (*.json)|*.json"),
		EFileDialogFlags::None,
		paths);
	if (!selected || paths.Num() < 1) {
		return;
	}

	// Get package path
	/*
	const FName PackagePath = GetPackagePath();
	UE_LOG(LoaderPlugin, Log, TEXT("PackagePath: %s"), *PackagePath.ToString());
	*/
	
	IAssetTools& AssetTools = FAssetToolsModule::GetModule().Get();

	TArray<FString> ImportFiles;
	ImportFiles.Add(TEXT("C:\\VRtemp\\import_SM_0.obj"));
	ImportFiles.Add(TEXT("C:\\VRtemp\\import_Diffuse_0.png"));
	ImportFiles.Add(TEXT("C:\\VRtemp\\Hachi.wav"));
	AssetTools.ImportAssets(ImportFiles, TEXT("/Game/AutoLoaded"));
	
	return;

	FString selected_path = paths[0];
	UE_LOG(LoaderPlugin, Log, TEXT("Loading scan directory %s"), *selected_path);
	
	const std::string file_path(TCHAR_TO_UTF8(*selected_path));

	picojson::object scene_root = LoadJsonFromFile(file_path).get<picojson::object>();
	auto lights = scene_root["lights"].get<picojson::array>();
	UE_LOG(LoaderPlugin, Log, TEXT("* Number of Lights: %d"), lights.size());

	const float uu_per_meter = 100;
	FVector offset(0, 0, 2.0);
	
	for (auto& light : lights) {
		auto pos = light.get<picojson::object>()["pos"].get<picojson::object>();
		FVector location(pos["x"].get<double>(), pos["y"].get<double>(), pos["z"].get<double>());
		FTransform pose((location + offset) * uu_per_meter);
		InsertAssetToScene(pose, "/Script/Engine.PointLight");
	}

	// Insert exterior mesh
	FTransform pose(FQuat(0, 0, 0, 1), offset * uu_per_meter);
	AActor* actor = InsertAssetToScene(pose, "/Game/Auto/Object.Object");
	if (!actor) {
		UE_LOG(LoaderPlugin, Error, TEXT("Failed to insert exterior mesh"));
		return;
	}
	auto* component = actor->FindComponentByClass<USceneComponent>();
	if (component) {
		component->SetWorldScale3D(FVector(uu_per_meter, uu_per_meter, uu_per_meter));
	}

	// Reference: https://wiki.unrealengine.com/Procedural_Mesh_Generation
	for (const auto& object : scene_root["objects"].get<picojson::array>()) {
		const std::string oid = object.get<std::string>();
		const std::string name = "flat_poly_" + oid + "object";
		const std::string asset_path = "/Game/Auto/" + name + "." + name;

		// Dunno why, but specifying scale in CreateActor is being ignored. Set it after actor is created.
		FTransform pose(FQuat(0, 0, 0, 1), offset * uu_per_meter);
		AActor* actor = InsertAssetToScene(pose, asset_path);
		if (!actor) {
			UE_LOG(LoaderPlugin, Warning, TEXT("Failed to insert %s, ignoring"), widen(asset_path).c_str());
			continue;
		}
		auto* component = actor->FindComponentByClass<USceneComponent>();
		if (!component) {
			UE_LOG(LoaderPlugin, Error, TEXT("Couldn't set actor mobility to movable"));
			return;
		}
		component->SetMobility(EComponentMobility::Movable);
		component->SetWorldScale3D(FVector(uu_per_meter, uu_per_meter, uu_per_meter));
		
		auto* mesh = actor->FindComponentByClass<UStaticMeshComponent>();
		if (!mesh) {
			UE_LOG(LoaderPlugin, Error, TEXT("Couldn't get StaticMeshComponent of inserted actor"));
			return;
		}
		// Reference: https://forums.unrealengine.com/archive/index.php/t-2078.html
		/*
		UBodySetup* bs = mesh->StaticMesh->BodySetup;
		if (!bs) {
			UE_LOG(LoaderPlugin, Error, TEXT("Couldn't get StaticMeshComponent->BodySetup of inserted actor"));
			return;
		}
		*/
		GenerateBoxAsSimpleCollision(mesh->StaticMesh);
		//bs->CollisionTraceFlag = CTF_UseComplexAsSimple;
		//bs->bMeshCollideAll = true;
		//bs->InvalidatePhysicsData();
		//bs->CreatePhysicsMeshes();
		auto* prim = actor->FindComponentByClass<UPrimitiveComponent>();
		if (!prim) {
			UE_LOG(LoaderPlugin, Error, TEXT("Couldn't set simulate physics flag"));
			return;
		}
		prim->SetSimulatePhysics(true);
		actor->SetActorEnableCollision(true);
	}
#undef LOCTEXT_NAMESPACE
}

picojson::value FLoaderPlugin::LoadJsonFromFile(const std::string& path) {
	picojson::value root;
	try {
		std::ifstream test(path);
		test >> root;
	}
	catch (...) {
		UE_LOG(LoaderPlugin, Warning, TEXT("Failed to load text; aborting import"));
	}
	return root;
}

AActor* FLoaderPlugin::InsertAssetToScene(FTransform pose, const std::string& asset_path) {
	UObject* asset = StaticLoadObject(UObject::StaticClass(), nullptr, widen(asset_path).c_str());
	if (asset == nullptr) {
		UE_LOG(LoaderPlugin, Error, TEXT("Failed to load asset %s"), widen(asset_path).c_str());
		return nullptr;
	}
	auto* factory = FActorFactoryAssetProxy::GetFactoryForAssetObject(asset);
	auto* level = GEditor->GetEditorWorldContext().World()->GetCurrentLevel();
	assert(factory != nullptr);
	assert(level != nullptr);
	return factory->CreateActor(asset, level, pose);
}

void FLoaderPlugin::AddToolbarExtension(FToolBarBuilder& builder) {
#define LOCTEXT_NAMESPACE "LevelEditorToolBar"
	UE_LOG(LoaderPlugin, Log, TEXT("Adding button"));
	FSlateIcon IconBrush = FSlateIcon(FEditorStyle::GetStyleSetName(), "LevelEditor.ViewOptions", "LevelEditor.ViewOptions.Small");

	builder.AddToolBarButton(LoaderPluginCommands::Get().loadButton, NAME_None,
		LOCTEXT("LoadButton", "Import Earthquake"),
		LOCTEXT("LoadButton_ToolTip", "Import earthquake simulation objects from reconstruction data"), IconBrush, NAME_None);
#undef LOCTEXT_NAMESPACE
}

void FLoaderPlugin::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	UE_LOG(LoaderPlugin, Log, TEXT("EqExperiment asset loader plugin un-initializing"));
}

IMPLEMENT_MODULE(FLoaderPlugin, LoaderPlugin)
