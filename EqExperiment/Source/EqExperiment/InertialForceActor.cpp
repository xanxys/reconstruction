#include "EqExperiment.h"
#include "InertialForceActor.h"

#include "InertialForceComponent.h"


AInertialForceActor::AInertialForceActor(const class FPostConstructInitializeProperties& PCIP)
: Super(PCIP)
{
	ForceComponent = PCIP.CreateDefaultSubobject<UInertialForceComponent>(this, TEXT("ForceComponent0"));

#if WITH_EDITOR
	SpriteComponent = PCIP.CreateEditorOnlyDefaultSubobject<UBillboardComponent>(this, TEXT("Sprite"));
	if (SpriteComponent)
	{
		// Structure to hold one-time initialization
		if (!IsRunningCommandlet())
		{
			struct FConstructorStatics
			{
				ConstructorHelpers::FObjectFinderOptional<UTexture2D> RadialForceTexture;
				FName ID_Physics;
				FText NAME_Physics;
				FConstructorStatics()
					: RadialForceTexture(TEXT("/Engine/EditorResources/S_RadForce.S_RadForce"))
					, ID_Physics(TEXT("Physics"))
					, NAME_Physics(NSLOCTEXT("SpriteCategory", "Physics", "Physics"))
				{
				}
			};
			static FConstructorStatics ConstructorStatics;

			SpriteComponent->Sprite = ConstructorStatics.RadialForceTexture.Get();

#if WITH_EDITORONLY_DATA
			SpriteComponent->SpriteInfo.Category = ConstructorStatics.ID_Physics;
			SpriteComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Physics;
#endif // WITH_EDITORONLY_DATA
		}

		SpriteComponent->RelativeScale3D.X = 0.5f;
		SpriteComponent->RelativeScale3D.Y = 0.5f;
		SpriteComponent->RelativeScale3D.Z = 0.5f;
		SpriteComponent->AttachParent = ForceComponent;
		SpriteComponent->bIsScreenSizeScaled = true;
	}
#endif

	RootComponent = ForceComponent;
	SetRemoteRoleForBackwardsCompat(ROLE_SimulatedProxy);
	bReplicates = true;
	bAlwaysRelevant = true;
	NetUpdateFrequency = 0.1f;
}

#if WITH_EDITOR
void AInertialForceActor::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
	FVector ModifiedScale = DeltaScale * (AActor::bUsePercentageBasedScaling ? 500.0f : 5.0f);

	const float Multiplier = (ModifiedScale.X > 0.0f || ModifiedScale.Y > 0.0f || ModifiedScale.Z > 0.0f) ? 1.0f : -1.0f;
	if (ForceComponent.IsValid())
	{
		ForceComponent->Radius += Multiplier * ModifiedScale.Size();
		ForceComponent->Radius = FMath::Max(0.f, ForceComponent->Radius);
	}
}
#endif
