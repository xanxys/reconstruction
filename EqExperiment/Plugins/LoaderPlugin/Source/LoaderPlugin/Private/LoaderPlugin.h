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

	// Load experiment data.
	void UnpackExperiment(const std::string& dir_path);

	// Load scene data.
	void UnpackScene(const std::string& dir_path);
	
	// Strip file name.
	// C:\foo\bar.txt -> C:\foo\
	// behavior undefined for directories
	static std::string dirname(const std::string& path);
	
	picojson::value LoadJsonFromFile(const std::string& path);
	Json::Value LoadJsonFromFileNew(const std::string& path);

	AActor* InsertAssetToScene(FTransform pose, const std::string& asset_path);
private:
	const float assumed_scale = 100;  // uu/meter
	static const std::string PathSplitter;
	TSharedPtr<FUICommandList> commands;
};
