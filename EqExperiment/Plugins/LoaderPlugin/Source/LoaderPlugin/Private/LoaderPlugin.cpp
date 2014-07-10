// Copyright 1998-2014 Epic Games, Inc. All Rights Reserved.

#include "LoaderPluginPrivatePCH.h"
#include "LoaderPlugin.h"

#include "ModuleManager.h"
#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Slate.h"

#include "LoaderPluginCommands.h"


void FLoaderPlugin::StartupModule()
{
	// This code will execute after your module is loaded into memory (but after global variables are initialized, of course.)
	UE_LOG(LoaderPlugin, Log, TEXT("EqExperiment asset loader plugin initializing"));

	LoaderPluginCommands::Register();

	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	TSharedPtr<FExtender> Extenders = LevelEditorModule.GetToolBarExtensibilityManager()->GetAllExtenders();

	TSharedPtr<FExtender> MyExtender = MakeShareable(new FExtender);
	MyExtender->AddToolBarExtension("Settings", EExtensionHook::After, nullptr,
		FToolBarExtensionDelegate::CreateRaw(this, &FLoaderPlugin::AddToolbarExtension));

	LevelEditorModule.GetToolBarExtensibilityManager()->AddExtender(MyExtender);
}

void FLoaderPlugin::AddToolbarExtension(FToolBarBuilder& builder) {
#define LOCTEXT_NAMESPACE "LevelEditorToolBar"
	UE_LOG(LoaderPlugin, Log, TEXT("Adding button"));
	FSlateIcon IconBrush = FSlateIcon(FEditorStyle::GetStyleSetName(), "LevelEditor.ViewOptions", "LevelEditor.ViewOptions.Small");
	builder.AddToolBarButton(LoaderPluginCommands::Get().MyButton, NAME_None,
		LOCTEXT("MyButton", "My Button"),
		LOCTEXT("MyButton_ToolTip", "Click me to display a message"), IconBrush, NAME_None);
#undef LOCTEXT_NAMESPACE
}

void FLoaderPlugin::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

IMPLEMENT_MODULE(FLoaderPlugin, LoaderPlugin)
