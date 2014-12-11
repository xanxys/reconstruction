

#pragma once

#include "GameFramework/Actor.h"
#include "NoisyActor.generated.h"

/**
 * ANoisyActor
 *    |-BoxComponent (for physics, invisible)
           |-StaticMeshComponent (for rendering, visible, no-physics)

 */
UCLASS()
class EQEXPERIMENT_API ANoisyActor : public AActor
{
	GENERATED_UCLASS_BODY()

	virtual void BeginPlay() override;

	TSubobjectPtr<UStaticMeshComponent> StaticMeshComponent;
	//TSubobjectPtr<UBoxComponent> BoxComponent;
};
