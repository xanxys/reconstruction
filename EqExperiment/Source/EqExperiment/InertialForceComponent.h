#pragma once

#include <vector>

#include "InertialForceComponent.generated.h"

/**
*	Used to emit a radial force or impulse that can affect physics objects and or destructible objects.
*/
UCLASS(hidecategories = (Object, Mobility), ClassGroup = Physics, showcategories = Trigger, meta = (BlueprintSpawnableComponent), MinimalAPI)
class UInertialForceComponent : public USceneComponent
{
	GENERATED_UCLASS_BODY()

	/** The radius to apply the force or impulse in */
	UPROPERTY(interp, EditAnywhere, BlueprintReadWrite, Category = RadialForceComponent)
	float Radius;
		
	/** Constant acceleration vs. temporally changing acceleration */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Force)
	bool IsConstant;

	/** Acceleration of objects inside spherical field. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Force)
	FVector Acceleration;

	/** Add an object type for this radial force to affect */
	UFUNCTION(BlueprintCallable, Category = "Physics|Components|InertialForce")
		virtual void AddObjectTypeToAffect(TEnumAsByte<enum EObjectTypeQuery> ObjectType);

	/** Remove an object type that is affected by this radial force */
	UFUNCTION(BlueprintCallable, Category = "Physics|Components|InertialForce")
		virtual void RemoveObjectTypeToAffect(TEnumAsByte<enum EObjectTypeQuery> ObjectType);

	/** Add a collision channel for this radial force to affect */
	void AddCollisionChannelToAffect(enum ECollisionChannel CollisionChannel);

	/** Set acceleration profile as linearly interpolated table. DeltaT should be smaller than tick duration.
	Accels.size() >= 2 and DeltaT > 0 must be satisfied.
	*/
	void SetAccelerationProfile(const std::vector<FVector>& Accels, float DeltaT);

	// Start playing back acceleration. If IsConstant, nothing will happen.
	// If it's already playing, restart from the beginning.
	// After finishing, the last acceleration will be played.
	void StartPlaying();

protected:
	/** The object types that are affected by this radial force */
	UPROPERTY(EditAnywhere, Category = RadialForceComponent)
		TArray<TEnumAsByte<enum EObjectTypeQuery>> ObjectTypesToAffect;

	/** Cached object query params derived from ObjectTypesToAffect */
	FCollisionObjectQueryParams CollisionObjectQueryParams;

	// Dynamic acceleration profile.
	float AccelerationTableResolution;
	std::vector<FVector> AccelerationTable;

	// Dynamic acceleration playback status.
	float AccelerationPlaybackT0;

protected:
	// Begin UActorComponent interface.
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
	// End UActorComponent interface.

	// Begin UObject interface.
	virtual void PostLoad() override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	// End UObject interface.

	/** Update CollisionObjectQueryParams from ObjectTypesToAffect */
	void UpdateCollisionObjectQueryParams();
};



