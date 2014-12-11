

#include "EqExperiment.h"
#include "NoisyActor.h"


ANoisyActor::ANoisyActor(const class FPostConstructInitializeProperties& PCIP)
	: Super(PCIP)
{
	const float uu_per_meter = 100;

	StaticMeshComponent = PCIP.CreateAbstractDefaultSubobject<UStaticMeshComponent>(this, TEXT("staticmesh"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> StaticMesh(TEXT("StaticMesh'/Game/Meshes/TemplateCube_Rounded.TemplateCube_Rounded'"));
	static ConstructorHelpers::FObjectFinder<UMaterial> Material(TEXT("MaterialInstanceConstant'/Game/Materials/M_Basic_Floor.M_Basic_Floor'"));
	StaticMeshComponent->SetStaticMesh(StaticMesh.Object);
	StaticMeshComponent->SetMaterial(0, Material.Object);
	StaticMeshComponent->SetMobility(EComponentMobility::Movable);
		
	/*
	BoxComponent = PCIP.CreateAbstractDefaultSubobject<UBoxComponent>(this, TEXT("box"));
	BoxComponent->InitBoxExtent(FVector(100, 100, 100));
	StaticMeshComponent->AttachTo(BoxComponent);
	*/

	StaticMeshComponent->SetMobility(EComponentMobility::Movable);
	StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	StaticMeshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
	
		
	PrimaryActorTick.bCanEverTick = true;

	StaticMeshComponent->SetWorldLocation(FVector(100, 100, 1000));
	StaticMeshComponent->SetPhysicsLinearVelocity(FVector(0, 0, 0));
	

}

void ANoisyActor::BeginPlay() {
	Super::BeginPlay();
	StaticMeshComponent->SetSimulatePhysics(true);
	StaticMeshComponent->WakeRigidBody();
}
