#pragma once

#include "ModuleManager.h"
#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Slate.h"

#include "picojson.h"
#include "json/json.h"

#include <string>


DECLARE_LOG_CATEGORY_EXTERN(LoaderPlugin, Log, All);
DEFINE_LOG_CATEGORY(LoaderPlugin);class FLoaderPlugin : public ILoaderPlugin
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	void OnLoadButtonClicked();
	void AddToolbarExtension(FToolBarBuilder& builder);

	//FName GetPackagePath();

	picojson::value LoadJsonFromFile(const std::string& path);
	Json::Value LoadJsonFromFileNew(const std::string& path);

	AActor* InsertAssetToScene(FTransform pose, const std::string& asset_path);
private:
	TSharedPtr<FUICommandList> commands;
};
