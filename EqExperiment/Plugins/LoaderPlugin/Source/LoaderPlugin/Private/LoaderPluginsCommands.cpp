#include "LoaderPluginPrivatePCH.h"
#include "LoaderPluginCommands.h"


LoaderPluginCommands::LoaderPluginCommands() :	TCommands<LoaderPluginCommands>(
		TEXT("LoaderPlugin"), NSLOCTEXT("Contexts", "LoaderPlugin", "Hamads Plugin"), NAME_None, FEditorStyle::GetStyleSetName()) {
}

void LoaderPluginCommands::RegisterCommands() {
	UI_COMMAND(MyButton, "My Hamad", "Displays a message in output log", EUserInterfaceActionType::Button, FInputGesture());
}
