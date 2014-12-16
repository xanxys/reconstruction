

#pragma once

#include <set>
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

	static FTransform DeserializeTransform(const Json::Value& Trans);
private:
	std::set<std::string> ActorNames;

	Json::Value RuntimeInfo;
	// must be kept in sync with LoaderPlugin.h
	const std::string RuntimeInfoPath = "C:\\VR14\\runtime.json";
};
