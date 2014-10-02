#include "LoaderPluginPrivatePCH.h"
#include "LoaderPlugin.h"

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <locale>

#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Editor/UnrealEd/Public/AssetSelection.h"
#include "ModuleManager.h"
#include "picojson.h"
#include "Runtime/CoreUObject/Public/UObject/UObjectGlobals.h"
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

	LevelEditorModule.GetToolBarExtensibilityManager()->AddExtender(MyExtender);
}

// Very helpful answer
// https://forums.unrealengine.com/showthread.php?22023-UE4-Automatic-Level-Builder-Construction-and-Pipeline-Scripts&p=103983&viewfull=1#post103983
void FLoaderPlugin::OnLoadButtonClicked() {
	UE_LOG(LoaderPlugin, Log, TEXT("Clicked"));
	
	const std::string file_path = "\\\\LITHIUM\\public\\research\\2014\\reconstruction\\reconstruction-generated-c082e271\\test-20140801-1524-gakusei-table\\small_data.json";

	picojson::object scene_root = LoadJsonFromFile(file_path).get<picojson::object>();
	auto lights = scene_root["lights"].get<picojson::array>();
	UE_LOG(LoaderPlugin, Log, TEXT("* Number of Lights: %d"), lights.size());

	const float uu_per_meter = 100;
	FVector offset(0, 0, 1.0);
	
	for (auto& light : lights) {
		auto pos = light.get<picojson::object>()["pos"].get<picojson::object>();
		FVector location(pos["x"].get<double>(), pos["y"].get<double>(), pos["z"].get<double>());
		FTransform pose((location + offset) * uu_per_meter);
		InsertAssetToScene(pose, "/Script/Engine.PointLight");
	}

	// Reference: https://wiki.unrealengine.com/Procedural_Mesh_Generation
	for(int i = 0; i < 5; i++) {
		const std::string name = "flat_poly_" + std::to_string(i) + "object";
		const std::string asset_path = "/Game/Auto/" + name + "." + name;

		// Dunno why, but specifying scale in CreateActor is being ignored. Set it after actor is created.
		FTransform pose(FQuat(0, 0, 0, 1), offset * uu_per_meter);
		AActor* actor = InsertAssetToScene(pose, asset_path);
		auto* component = actor->FindComponentByClass<USceneComponent>();
		if (component) {
			component->SetMobility(EComponentMobility::Movable);
			component->SetWorldScale3D(FVector(uu_per_meter, uu_per_meter, uu_per_meter));
		}
		else {
			UE_LOG(LoaderPlugin, Error, TEXT("Couldn't set actor mobility to movable"));
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
