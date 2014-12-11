

#include "EqExperiment.h"
#include "NoisyActor.h"


ANoisyActor::ANoisyActor(const class FPostConstructInitializeProperties& PCIP)
	: Super(PCIP)
{
	const float uu_per_meter = 100;

	StaticMeshComponent = PCIP.CreateAbstractDefaultSubobject<UStaticMeshComponent>(this, TEXT("staticmesh"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> StaticMesh(TEXT("StaticMesh'/Game/Props/SM_Lamp_Wall.SM_Lamp_Wall'"));
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

	// Setup Collision
	UBodySetup* bs = StaticMeshComponent->StaticMesh->BodySetup;

	// should be given by creator
	FVector Center(0, 0, 0);
	FVector Extents(100, 100, 100);

	bs->Modify();
	bs->InvalidatePhysicsData();

	FKBoxElem BoxElem;
	BoxElem.Center = Center;
	BoxElem.X = Extents.X * 2.0f;
	BoxElem.Y = Extents.Y * 2.0f;
	BoxElem.Z = Extents.Z * 2.0f;
	bs->AggGeom.BoxElems.Add(BoxElem);

	StaticMeshComponent->RecreatePhysicsState();
	StaticMeshComponent->StaticMesh->MarkPackageDirty();
}


void ANoisyActor::BeginPlay() {
	Super::BeginPlay();
	StaticMeshComponent->SetSimulatePhysics(true);
	StaticMeshComponent->WakeRigidBody();
}
