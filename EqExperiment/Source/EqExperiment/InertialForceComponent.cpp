#include "EqExperiment.h"
#include "InertialForceComponent.h"

#include "GameFramework/MovementComponent.h"
#include "Net/UnrealNetwork.h"

//////////////////////////////////////////////////////////////////////////
// RADIALFORCECOMPONENT
UInertialForceComponent::UInertialForceComponent(const class FPostConstructInitializeProperties& PCIP)
: Super(PCIP)
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	Radius = 200.0f;
	Acceleration = FVector(980, 0, 0);

	// by default we affect all 'dynamic' objects that can currently be affected by forces
	AddCollisionChannelToAffect(ECC_Pawn);
	AddCollisionChannelToAffect(ECC_PhysicsBody);
	AddCollisionChannelToAffect(ECC_Vehicle);
	AddCollisionChannelToAffect(ECC_Destructible);

	UpdateCollisionObjectQueryParams();
}

void UInertialForceComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bIsActive)
	{
		const FVector Origin = GetComponentLocation();

		// Find objects within the sphere
		static FName AddForceOverlapName = FName(TEXT("AddForceOverlap"));
		TArray<FOverlapResult> Overlaps;

		FCollisionQueryParams Params(AddForceOverlapName, false);
		Params.bTraceAsyncScene = true; // want to hurt stuff in async scene
		GetWorld()->OverlapMulti(Overlaps, Origin, FQuat::Identity, FCollisionShape::MakeSphere(Radius), Params, CollisionObjectQueryParams);

		// Calculate current acceleration.
		FVector AccelNow;
		if (IsConstant) {
			AccelNow = Acceleration;
		}
		else {
			if (!World) {
				return;
			}
			const float ix = (World->GetTimeSeconds() - AccelerationPlaybackT0) / AccelerationTableResolution;
			if (ix <= 0) {
				AccelNow = AccelerationTable.front();
			}
			else if (ix >= AccelerationTable.size()) {
				AccelNow = AccelerationTable.back();
			}
			else {
				const int ix0 = ix;
				const float frac = ix - ix0;
				AccelNow = AccelerationTable[ix0] * (1 - frac) + AccelerationTable[ix0 + 1] * frac;
			}
		}

		// Iterate over each and apply force
		for (int32 OverlapIdx = 0; OverlapIdx<Overlaps.Num(); OverlapIdx++)
		{
			UPrimitiveComponent* PokeComp = Overlaps[OverlapIdx].Component.Get();
			if (PokeComp)
			{
				PokeComp->AddForce(PokeComp->GetMass() * AccelNow, NAME_None);

				// see if this is a target for a movement component
				/*
				AActor* PokeOwner = PokeComp->GetOwner();
				if (PokeOwner)
				{
					TArray<UMovementComponent*> MovementComponents;
					PokeOwner->GetComponents<UMovementComponent>(MovementComponents);
					for (const auto& MovementComponent : MovementComponents)
					{
						if (MovementComponent->UpdatedComponent == PokeComp)
						{
							MovementComponent->AddForce(ForceStrength, NAME_None);
							break;
						}
					}
				}
				*/
			}
		}
	}
}

void UInertialForceComponent::SetAccelerationProfile(const std::vector<FVector>& Accels, float DeltaT) {
	if (DeltaT <= 0 || Accels.size() < 2) {
		return;
	}
	AccelerationTable = Accels;
	AccelerationTableResolution = DeltaT;
}

void UInertialForceComponent::StartPlaying() {
	if (!World) {
		return;
	}

	AccelerationPlaybackT0 = World->GetTimeSeconds();
}

void UInertialForceComponent::PostLoad()
{
	Super::PostLoad();
	UpdateCollisionObjectQueryParams();
}

void UInertialForceComponent::AddCollisionChannelToAffect(enum ECollisionChannel CollisionChannel)
{
	EObjectTypeQuery ObjectType = UEngineTypes::ConvertToObjectType(CollisionChannel);
	if (ObjectType != ObjectTypeQuery_MAX)
	{
		AddObjectTypeToAffect(ObjectType);
	}
}

void UInertialForceComponent::AddObjectTypeToAffect(TEnumAsByte<enum EObjectTypeQuery> ObjectType)
{
	ObjectTypesToAffect.AddUnique(ObjectType);
	UpdateCollisionObjectQueryParams();
}

void UInertialForceComponent::RemoveObjectTypeToAffect(TEnumAsByte<enum EObjectTypeQuery> ObjectType)
{
	ObjectTypesToAffect.Remove(ObjectType);
	UpdateCollisionObjectQueryParams();
}

#if WITH_EDITOR

void UInertialForceComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// If we have edited the object types to effect, update our bitfield.
	if (PropertyChangedEvent.Property && PropertyChangedEvent.Property->GetFName() == TEXT("ObjectTypesToAffect"))
	{
		UpdateCollisionObjectQueryParams();
	}
}

#endif

void UInertialForceComponent::UpdateCollisionObjectQueryParams()
{
	CollisionObjectQueryParams = FCollisionObjectQueryParams(ObjectTypesToAffect);
}

