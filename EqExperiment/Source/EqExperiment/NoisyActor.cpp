#include "EqExperiment.h"
#include "NoisyActor.h"

#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/PhysicsEngine/BodySetup.h"
#include <vector>

template <typename ObjClass>
static ObjClass* LoadObjFromPath(const FName& Path)
{
	if (Path == NAME_None) return NULL;
	//~

	return Cast<ObjClass>(StaticLoadObject(ObjClass::StaticClass(), NULL, *Path.ToString()));
}

std::wstring widen(const std::string& s) {
	const int n_wstring = MultiByteToWideChar(CP_UTF8, 0, s.data(), s.size(), nullptr, 0);
	if (n_wstring == 0) {
		throw std::runtime_error("MultiByteToWideChar failed");
	}

	std::vector<wchar_t> buffer(n_wstring);
	const int n_written = MultiByteToWideChar(CP_UTF8, 0, s.data(), s.size(), buffer.data(), buffer.size());
	return std::wstring(buffer.begin(), buffer.end());
}

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
	StaticMeshComponent->SetNotifyRigidBodyCollision(true);
	
		
	PrimaryActorTick.bCanEverTick = true;

	StaticMeshComponent->SetWorldLocation(FVector(100, 100, 1000));
	StaticMeshComponent->SetPhysicsLinearVelocity(FVector(0, 0, 0));

	UpdateStaticMeshCollision();

	// set collision handler (for playing sounds)
	StaticMeshComponent->OnComponentHit.AddDynamic(this, &ANoisyActor::OnHit);

	// Lookup sound.
	for (int i = 0; i < 8; i++) {
		ConstructorHelpers::FObjectFinder<USoundWave> Sound(
			*FString::Printf(TEXT("/Game/Audio/collision-%d.collision-%d"), i + 1, i + 1));
		if (!Sound.Object) {
			continue;
		}
		hit_sounds.push_back(Sound.Object);
	}
	
}

void ANoisyActor::UpdateStaticMeshCollision() {
	// Setup Collision
	UBodySetup* bs = StaticMeshComponent->StaticMesh->BodySetup;

	// should be given by creator
	FVector Center(0, 0, 0);
	FVector Extents(100, 100, 100);

	bs->Modify();
	bs->InvalidatePhysicsData();  // comment this out to make "launch" & "cooking" pass. but without this, physics is a bit strange??

	FKBoxElem BoxElem;
	BoxElem.Center = Center;
	BoxElem.X = Extents.X * 2.0f;
	BoxElem.Y = Extents.Y * 2.0f;
	BoxElem.Z = Extents.Z * 2.0f;
	bs->AggGeom.BoxElems.Add(BoxElem);

	StaticMeshComponent->RecreatePhysicsState();
	StaticMeshComponent->StaticMesh->MarkPackageDirty();
}

void ANoisyActor::LoadInterior(const std::string& name) {
	UStaticMesh* sm = LoadObjFromPath<UStaticMesh>(widen("StaticMesh'/Game/Props/" + name + "." + name + "'").c_str());
	if (!sm) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("StaticMesh not found"));
		return;
	}
	StaticMeshComponent->SetStaticMesh(sm);
	UpdateStaticMeshCollision();
}

void ANoisyActor::LoadInteriorFullPath(const std::string& name) {
	UStaticMesh* sm = LoadObjFromPath<UStaticMesh>(widen(name).c_str());
	if (!sm) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("StaticMesh not found"));
		return;
	}
	StaticMeshComponent->SetStaticMesh(sm);
	UpdateStaticMeshCollision();
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
