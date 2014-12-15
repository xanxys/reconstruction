

#pragma once

#include <string>

#include "json/json.h"
#include "GameFramework/GameMode.h"
#include "EqSimGameMode.generated.h"

/**
 * 
 */
UCLASS()
class EQEXPERIMENT_API AEqSimGameMode : public AGameMode
{
	GENERATED_UCLASS_BODY()

	void BeginPlay() override;


	void PrepareSubExperiment(int SubExperimentId);
	
	USoundBase* EqBgSound;
private:
	// Spawn interior objects as Actors.
	// DO NOT CALL THIS TWICE.
	void SpawnInteriorObjects();

	// Reset all interior object poses.
	void ResetInteriorObjects();
private:
	Json::Value RuntimeInfo;
	// must be kept in sync with LoaderPlugin.h
	const std::string RuntimeInfoPath = "C:\\VR14\\runtime.json";
};
