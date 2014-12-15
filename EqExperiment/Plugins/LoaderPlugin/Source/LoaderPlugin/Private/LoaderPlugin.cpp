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
#include "Runtime/CoreUObject/Public/UObject/UObjectGlobals.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Editor/MainFrame/Public/Interfaces/IMainFrameModule.h"
#include "Slate.h"

#include "json/json.h"

#include "LoaderPluginCommands.h"

const std::string FLoaderPlugin::PathSplitter = "\\";
const std::string FLoaderPlugin::AltPathSplitter = "/";

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

	// Read specified path.
	try {
		const std::string file_path(TCHAR_TO_UTF8(*paths[0]));
		UnpackExperiment(dirname(file_path));
	}
	catch (const std::exception& exc) {
		UE_LOG(LoaderPlugin, Warning, TEXT("Exception: %s"), widen(exc.what()).c_str());
	}
#undef LOCTEXT_NAMESPACE
}

void FLoaderPlugin::UnpackExperiment(const std::string& dir_path) {
	UE_LOG(LoaderPlugin, Log, TEXT("Unpacking experiment package %s"), widen(dir_path).c_str());
	const Json::Value meta = LoadJsonFromFileNew(join(dir_path, "meta.json"));

	UnpackScene(join(dir_path, "scene"));
}

void FLoaderPlugin::UnpackScene(const std::string& dir_path) {
	UE_LOG(LoaderPlugin, Log, TEXT("Unpacking scene asset %s"), widen(dir_path).c_str());
	const Json::Value meta = LoadJsonFromFileNew(join(dir_path, "meta.json"));

	// Check if assumed scale and exported scale are similar enough.
	if (std::abs(1 - meta["unit_per_meter"].asDouble() / assumed_scale) > 0.01) {
		UE_LOG(LoaderPlugin, Warning, TEXT("Export scale %f is too different from UU scale %f ."),
			meta["unit_per_meter"].asDouble(), assumed_scale);
		throw std::runtime_error("Wrong world scale");
	}

	// Get scene instantiation from TargetPoint.
	UWorld* const World = GEditor->GetEditorWorldContext().World();
	if (!World) {
		throw std::runtime_error("Couldn't get reference of UWorld");
	}
	
	AActor* RoomMarker = FindTargetPointByName(TargetRoom);
	if (!RoomMarker) {
		UE_LOG(LoaderPlugin, Warning, TEXT("You need to add TargetPoint with name %s to specify where to load scene"),
			widen(TargetRoom).c_str());
		throw std::runtime_error("RoomMarker not found");
	}
	const FVector RoomOrigin = RoomMarker->GetActorLocation();
	UE_LOG(LoaderPlugin, Log, TEXT("Instantiating at %s"), *RoomOrigin.ToString());
	
	FVector PlayerOrigin;
		
	
	/*
	IAssetTools& AssetTools = FAssetToolsModule::GetModule().Get();

	TArray<FString> ImportFiles;
	ImportFiles.Add(TEXT("C:\\VRtemp\\import_SM_0.obj"));
	ImportFiles.Add(TEXT("C:\\VRtemp\\import_Diffuse_0.png"));
	ImportFiles.Add(TEXT("C:\\VRtemp\\Hachi.wav"));
	AssetTools.ImportAssets(ImportFiles, TEXT("/Game/AutoLoaded"));
	*/

	UE_LOG(LoaderPlugin, Log, TEXT("* Number of Lights: %d"), meta["lights"].size());

	FVector offset(0, 0, 2.0);

	for (const auto& light : meta["lights"]) {
		const FVector location(
			light["pos"]["x"].asDouble(),
			light["pos"]["y"].asDouble(),
			light["pos"]["z"].asDouble());
		const FTransform pose(location + offset);
		InsertAssetToScene(pose, "/Script/Engine.PointLight");
	}


#if 0
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
#endif
}

AActor* FLoaderPlugin::FindTargetPointByName(const std::string& name) {
	UWorld* const World = GEditor->GetEditorWorldContext().World();
	if (!World) {
		return nullptr;
	}

	TArray<AActor*> Targets;
	UGameplayStatics::GetAllActorsOfClass(World, ATargetPoint::StaticClass(), Targets);
	for (AActor* Target : Targets) {
		if (TCHAR_TO_UTF8(*Target->GetName()) == name) {
			return Target;
		}
	}
	return nullptr;
}


std::string FLoaderPlugin::dirname(const std::string& path) {
	const std::string path_canon = canonicalize(path);
	const auto ix_split = path_canon.rfind(PathSplitter);
	if (ix_split == std::string::npos) {
		return path_canon;
	}
	else {
		return path_canon.substr(0, ix_split) + PathSplitter;
	}
}

std::string FLoaderPlugin::canonicalize(const std::string& path) {
	std::string result;

	int ix_curr = 0;
	while (true) {
		const auto ix = path.find(AltPathSplitter, ix_curr);
		if (ix == std::string::npos) {
			result += path.substr(ix_curr);
			break;
		}
		else {
			result += path.substr(ix_curr, ix - ix_curr);
			result += PathSplitter;
			ix_curr = ix + AltPathSplitter.size();
		}
	}
	return result;
}

std::string FLoaderPlugin::join(const std::string& path0, const std::string& path1) {
	const std::string pc0 = canonicalize(path0);
	const std::string pc1 = canonicalize(path1);
	if (pc0.size() < PathSplitter.size()) {
		return path1;
	}
	else {
		const bool ends_with_spl = pc0.substr(pc0.size() - PathSplitter.size(), PathSplitter.size()) == PathSplitter;
		if (ends_with_spl) {
			return pc0 + pc1;
		}
		else {
			return pc0 + PathSplitter + pc1;
		}
	}
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


Json::Value FLoaderPlugin::LoadJsonFromFileNew(const std::string& path) {
	try {
		Json::Value root;
		std::ifstream ifs(path);
		if (!ifs.is_open()) {
			throw std::runtime_error("Couldn't open " + path);
		}
		Json::Reader().parse(ifs, root);
		return root;
	}
	catch (...) {
		// UE_LOG(LoaderPlugin, Warning, TEXT("Failed to load text; aborting import"));
		throw std::runtime_error("Couldn't parse " + path);
	}
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
