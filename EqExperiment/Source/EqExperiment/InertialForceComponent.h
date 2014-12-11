// Copyright 1998-2014 Epic Games, Inc. All Rights Reserved.


#pragma once
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

protected:
	/** The object types that are affected by this radial force */
	UPROPERTY(EditAnywhere, Category = RadialForceComponent)
		TArray<TEnumAsByte<enum EObjectTypeQuery>> ObjectTypesToAffect;

	/** Cached object query params derived from ObjectTypesToAffect */
	FCollisionObjectQueryParams CollisionObjectQueryParams;

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



