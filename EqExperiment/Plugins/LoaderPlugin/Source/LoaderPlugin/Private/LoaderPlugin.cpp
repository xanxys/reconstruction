#include "LoaderPluginPrivatePCH.h"
#include "LoaderPlugin.h"

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>

#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Editor/UnrealEd/Public/AssetSelection.h"
#include "ModuleManager.h"
#include "picojson.h"
#include "Runtime/CoreUObject/Public/UObject/UObjectGlobals.h"
#include "Slate.h"

#include "LoaderPluginCommands.h"


void FLoaderPlugin::StartupModule()
{
	// This code will execute after your module is loaded into memory (but after global variables are initialized, of course.)
	UE_LOG(LoaderPlugin, Log, TEXT("EqExperiment asset loader plugin initializing"));

	LoaderPluginCommands::Register();

	commands = MakeShareable(new FUICommandList());
	commands->MapAction(
		LoaderPluginCommands::Get().loadButton,
		FExecuteAction::CreateRaw(this, &FLoaderPlugin::OnLoadButtonClicked),
		FCanExecuteAction());


	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
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
	std::ifstream test(file_path);
	picojson::value small_data;
	test >> small_data;
	
	std::stringstream ss;
	ss << small_data;
	std::string data = ss.str();
	UE_LOG(LoaderPlugin, Log, TEXT("Loaded: %s"), *FString(data.c_str()));
	auto lights = small_data.get<picojson::object>()["lights"].get<picojson::array>();
	UE_LOG(LoaderPlugin, Log, TEXT("* Number of Lights: %d"), lights.size());

	// TODO: put asset (StaticMesh) to project and scene

	const std::string asset_path = "/Script/Engine.PointLight";

	UObject* asset = StaticLoadObject(UObject::StaticClass(), nullptr, _T("/Script/Engine.PointLight"));
	if(asset == nullptr) {
		UE_LOG(LoaderPlugin, Error, TEXT("Failed to load point light asset"));
		return;
	}
	UE_LOG(LoaderPlugin, Log, TEXT("Asset loaded"));

	auto* factory = FActorFactoryAssetProxy::GetFactoryForAssetObject(asset);
	auto* level = GEditor->GetEditorWorldContext().World()->GetCurrentLevel();

	UE_LOG(LoaderPlugin, Log, TEXT("Creating Actor"));
	assert(factory != nullptr);
	assert(level != nullptr);
	
	const float uu_per_meter = 100;
	for (auto& light : lights) {
		auto pos = light.get<picojson::object>()["pos"].get<picojson::object>();
		FVector location(pos["x"].get<double>(), pos["y"].get<double>(), pos["z"].get<double>());
		location *= uu_per_meter;
		const FRotator rotation(0, 0, 0);
		UE_LOG(LoaderPlugin, Log, TEXT("Inserting Light at %s"), *location.ToString());
		AActor* actor = factory->CreateActor(asset, level, location, &rotation);
	}
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
