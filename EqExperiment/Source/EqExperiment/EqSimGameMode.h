

#pragma once

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
	
	USoundBase* EqBgSound;
};
