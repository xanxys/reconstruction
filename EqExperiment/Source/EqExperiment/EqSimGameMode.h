

#pragma once

#include <set>
#include <string>

#include "json/json.h"
#include "GameFramework/GameMode.h"
#include "EqSimGameMode.generated.h"

/**
 * 
 */
UCLASS(Blueprintable)
class EQEXPERIMENT_API AEqSimGameMode : public AGameMode
{
	GENERATED_UCLASS_BODY()

	void BeginPlay() override;

	// Reset all interior object poses.
	UFUNCTION(BlueprintCallable, Category="EqSim")
	void ResetInteriorObjects();

	void PrepareSubExperiment(int SubExperimentId);
	
	USoundBase* EqBgSound;
private:
	// Spawn interior objects as Actors.
	// DO NOT CALL THIS TWICE.
	void SpawnInteriorObjects();



	static FTransform DeserializeTransform(const Json::Value& Trans);
private:
	std::set<std::string> ActorNames;

	Json::Value RuntimeInfo;
	// must be kept in sync with LoaderPlugin.h
	const std::string RuntimeInfoPath = "C:\\VR14\\runtime.json";
};
