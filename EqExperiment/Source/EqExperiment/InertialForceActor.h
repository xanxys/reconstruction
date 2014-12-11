

#pragma once

#include "GameFramework/Actor.h"
#include "InertialForceActor.generated.h"

/**
 * 
 */
UCLASS()
class EQEXPERIMENT_API AInertialForceActor : public AActor
{
	GENERATED_UCLASS_BODY()

	/** Force component */
	UPROPERTY(Category = InertialForceActor, VisibleAnywhere, BlueprintReadOnly, meta = (ExposeFunctionCategories = "Activation,Components|Activation,Physics,Physics|Components|InertialForce"))
	TSubobjectPtr<class UInertialForceComponent> ForceComponent;

#if WITH_EDITORONLY_DATA
	UPROPERTY()
		TSubobjectPtr<UBillboardComponent> SpriteComponent;
#endif

#if WITH_EDITOR
	// Begin AActor interface.
	virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	// End AActor interface.
#endif
	
};
