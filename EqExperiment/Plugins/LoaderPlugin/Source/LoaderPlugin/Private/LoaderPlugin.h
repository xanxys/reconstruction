#pragma once

#include "ModuleManager.h"
#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Slate.h"

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

	// C:\foo bar.txt -> C:\foo\bar.txt
	// C:\foo\ bar.txt -> C:\foo\bar.txt
	// C:\foo\ test -> C:\foo\test
	static std::string join(const std::string& path0, const std::string& path1);

	// Convert all AltPathSplitter to PathSplitter
	static std::string canonicalize(const std::string& path);
	
	Json::Value LoadJsonFromFileNew(const std::string& path);

	AActor* InsertAssetToScene(FTransform pose, const std::string& asset_path);

	// Return nullptr when not found.
	AActor* FindTargetPointByName(const std::string& name);

	std::string GetFullPathForObjectSMAsset(const Json::Value& InteriorObj);

	static FTransform DeserializeTransform(const Json::Value& Trans);
	static Json::Value SerializeTransform(const FTransform& Trans);
private:
	const float assumed_scale = 100;  // uu/meter
	static const std::string PathSplitter;
	static const std::string AltPathSplitter;
	TSharedPtr<FUICommandList> commands;

	const std::string TargetRoom = "LoadMarker_Room";
	const std::string TargetPlayer = "LoadMarker_Player";
	const std::string AutoLoadAssetPath = "/Game/AutoLoaded";

	// must be kept in sync with EqSimGameMode.h
	const std::string RuntimeInfoPath = "C:\\VR14\\runtime.json";
};
