#pragma once

#include "ModuleManager.h"
#include "Editor/LevelEditor/Public/LevelEditor.h"
#include "Slate.h"

class LoaderPluginCommands : public TCommands<LoaderPluginCommands> {
public:
	LoaderPluginCommands();
	virtual void RegisterCommands() override;
	TSharedPtr<FUICommandInfo> MyButton;
};
