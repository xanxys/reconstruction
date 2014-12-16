

#pragma once

#include <string>
#include <vector>

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

	void LoadInteriorFullPath(const std::string& name);

	virtual void BeginPlay() override;

	TSubobjectPtr<UStaticMeshComponent> StaticMeshComponent;
protected:

	UFUNCTION()
	virtual void OnHit(AActor* OtherActor, UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit);

	std::vector<USoundBase*> hit_sounds;

	void UpdateStaticMeshCollision();
};
