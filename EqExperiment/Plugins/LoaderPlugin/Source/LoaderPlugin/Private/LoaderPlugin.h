#pragma once

#include "ModuleManager.h"
#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Slate.h"


DECLARE_LOG_CATEGORY_EXTERN(LoaderPlugin, Log, All);
DEFINE_LOG_CATEGORY(LoaderPlugin);class FLoaderPlugin : public ILoaderPlugin
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	void AddToolbarExtension(FToolBarBuilder& builder);
};
