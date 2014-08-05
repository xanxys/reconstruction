#include "LoaderPluginPrivatePCH.h"
#include "LoaderPlugin.h"

#include <cassert>
#include <fstream>
#include <string>

#include "ModuleManager.h"
#include "Editor/UnrealEd/Public/AssetSelection.h"
#include "Editor/LevelEditor/Public/LevelEditor.h"
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

	std::ifstream test("\\\\LITHIUM\\public\\research\\2014\\reconstruction\\reconstruction-generated-c082e271\\test-20140801-1524-gakusei-table\\small_data.json");
	std::string data;
	std::getline(test, data);
	UE_LOG(LoaderPlugin, Log, TEXT("Loaded: %s"), *FString(data.c_str()));

	// TODO: put asset (StaticMesh) to project and scene


	const std::string asset_path = "/Script/Engine.PointLight";

	UObject* asset = StaticLoadObject(UObject::StaticClass(), nullptr, _T("/Script/Engine.PointLight"));
	if(asset == nullptr) {
		UE_LOG(LoaderPlugin, Error, TEXT("Failed to load point light asset"));
		return;
	}
	UE_LOG(LoaderPlugin, Log, TEXT("Asset loaded"));

	const FVector location(0, 0, 0);
	const FRotator rotation(0, 0, 0);
	auto* factory = FActorFactoryAssetProxy::GetFactoryForAssetObject(asset);
	auto* level = GEditor->GetEditorWorldContext().World()->GetCurrentLevel();

	UE_LOG(LoaderPlugin, Log, TEXT("Creating Actor"));
	assert(factory != nullptr);
	assert(level != nullptr);
	AActor* actor = factory->CreateActor(asset, level, location, &rotation);
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
