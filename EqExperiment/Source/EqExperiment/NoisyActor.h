

#pragma once

#include "GameFramework/Actor.h"
#include "NoisyActor.generated.h"

/**
 * ANoisyActor
 *  |-StaticMeshComponent (for rendering, visible, physics is handled by bs->AggGeom)
 */
UCLASS()
class EQEXPERIMENT_API ANoisyActor : public AActor
{
	GENERATED_UCLASS_BODY()

	virtual void BeginPlay() override;

	TSubobjectPtr<UStaticMeshComponent> StaticMeshComponent;
	//TSubobjectPtr<UBoxComponent> BoxComponent;
protected:

	UFUNCTION()
	virtual void OnHit(AActor* OtherActor, UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit);

	USoundBase* hit_sound;
};
