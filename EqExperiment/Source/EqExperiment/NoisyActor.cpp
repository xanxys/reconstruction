#include "EqExperiment.h"
#include "NoisyActor.h"

#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/PhysicsEngine/BodySetup.h"
#include <vector>

ANoisyActor::ANoisyActor(const class FPostConstructInitializeProperties& PCIP)
	: Super(PCIP)
{
	const float uu_per_meter = 100;

	StaticMeshComponent = PCIP.CreateAbstractDefaultSubobject<UStaticMeshComponent>(this, TEXT("staticmesh"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> StaticMesh(TEXT("StaticMesh'/Game/Props/SM_Lamp_Wall.SM_Lamp_Wall'"));
	StaticMeshComponent->SetStaticMesh(StaticMesh.Object);
	StaticMeshComponent->SetMobility(EComponentMobility::Movable);
	StaticMeshComponent->SetMobility(EComponentMobility::Movable);
	StaticMeshComponent->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
	StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	StaticMeshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
	StaticMeshComponent->SetNotifyRigidBodyCollision(true);
	
		
	PrimaryActorTick.bCanEverTick = true;

	StaticMeshComponent->SetWorldLocation(FVector(100, 100, 1000));
	StaticMeshComponent->SetPhysicsLinearVelocity(FVector(0, 0, 0));

	// set collision handler (for playing sounds)
	StaticMeshComponent->OnComponentHit.AddDynamic(this, &ANoisyActor::OnHit);

	// Lookup sound.
	for (int i = 0; i < 30; i++) {
		// TODO: Fix this
		// WARNING: Encapsulation boundary breach!!
		// "AutoLoaded", 30, "collision_%d" shouldn't belong here
		ConstructorHelpers::FObjectFinder<USoundWave> Sound(
			*FString::Printf(TEXT("/Game/AutoLoaded/collision_%d.collision-%d"), i, i));
		if (!Sound.Object) {
			continue;
		}
		hit_sounds.push_back(Sound.Object);
	}

	RootComponent = StaticMeshComponent;
}

void ANoisyActor::LoadInteriorFullPath(const std::string& name) {
	UStaticMesh* StaticMesh = LoadObjFromPath<UStaticMesh>(widen(name).c_str());
	if (!StaticMesh) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("StaticMesh not found"));
		return;
	}
	StaticMeshComponent->SetStaticMesh(StaticMesh);

	// Redo collision settings just in case.
	StaticMeshComponent->SetMobility(EComponentMobility::Movable);
	StaticMeshComponent->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
	StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	StaticMeshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
	StaticMeshComponent->SetNotifyRigidBodyCollision(true);
}

void ANoisyActor::BeginPlay() {
	Super::BeginPlay();
	StaticMeshComponent->SetSimulatePhysics(true);
	StaticMeshComponent->WakeRigidBody();
}

void ANoisyActor::OnHit(AActor* OtherActor, UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit) {
	// GEngine->AddOnScreenDebugMessage(-1, 0.5, FColor::White, TEXT("Hit!"));
	UWorld* World = GetWorld();
	if (!World || hit_sounds.empty()) {
		return;
	}

	if (FMath::FRand() > 0.03) {
		return;
	}
	const int ix = FMath::FRandRange(0, hit_sounds.size() - 1);
	UGameplayStatics::PlaySoundAtLocation(World, hit_sounds[ix], Hit.ImpactPoint);
}
