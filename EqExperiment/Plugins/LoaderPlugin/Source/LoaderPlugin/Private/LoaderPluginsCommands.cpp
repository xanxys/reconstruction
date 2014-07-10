#include "LoaderPluginPrivatePCH.h"
#include "LoaderPluginCommands.h"


LoaderPluginCommands::LoaderPluginCommands() :	TCommands<LoaderPluginCommands>(
		TEXT("LoaderPlugin"), NSLOCTEXT("Contexts", "LoaderPlugin", "Hamads Plugin"), NAME_None, FEditorStyle::GetStyleSetName()) {
}

void LoaderPluginCommands::RegisterCommands() {
	UI_COMMAND(loadButton, "Load Earthquake", "Displays a message in output log (for now)", EUserInterfaceActionType::Button, FInputGesture());
}
