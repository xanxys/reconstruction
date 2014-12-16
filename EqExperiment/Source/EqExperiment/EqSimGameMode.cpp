#include "EqExperiment.h"
#include "EqSimGameMode.h"

#include <fstream>
#include <string>

#include "InertialForceActor.h"
#include "NoisyActor.h"


AEqSimGameMode::AEqSimGameMode(const class FPostConstructInitializeProperties& PCIP)
	: Super(PCIP)
{
	DefaultPawnClass = ConstructorHelpers::FClassFinder<APawn>(TEXT("/Game/Blueprints/MyCharacter")).Class;
}

void AEqSimGameMode::BeginPlay() {
	Super::BeginPlay();

	// Load critical runtime info from fucking FIXED PATH.
	{
		std::ifstream ifs(RuntimeInfoPath);
		if (!ifs.is_open()) {
			GEngine->AddOnScreenDebugMessage(-1, 10, FColor::Red, TEXT("runtime info not found; CANNOT CONTINUE!"));
			return;
		}
		Json::Reader().parse(ifs, RuntimeInfo);
	}
	SpawnInteriorObjects();
}

void AEqSimGameMode::PrepareSubExperiment(int32 SubExperimentId) {
	UWorld* const World = GetWorld();
	if (!World || RuntimeInfo.isNull()) {
		return;
	}

	// Add InertialForce gen.
	if (SubExperimentId < 1 || static_cast<int>(RuntimeInfo["quakes"].size()) < SubExperimentId) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red,
			*FString::Printf(TEXT("SubExperimentId=%d is out of range; cannot continue")));
	}

	StartQuake(RuntimeInfo["quakes"][SubExperimentId - 1]);

	GEngine->AddOnScreenDebugMessage(-1, 1, FColor::White, TEXT("Commencing SubExperiment"));
}

void AEqSimGameMode::AbortSubExperiment() {
	UWorld* const World = GetWorld();
	if (!World || RuntimeInfo.isNull()) {
		return;
	}

	GEngine->AddOnScreenDebugMessage(-1, 1, FColor::Red, TEXT("Aborting SubExperiment"));
	// Remove all InertialForce generator.
	for (auto It = TActorIterator<AInertialForceActor>(World); It; ++It) {
		It->Destroy();
	}
}

void AEqSimGameMode::StartQuake(const Json::Value& Quake) {
	const AActor* RoomMarker = FindTargetPointByName(TargetRoom);
	if (!GetWorld() || !RoomMarker) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("RoomMaker Not Found; cannot continue"));
		return;
	}
	const FVector RoomOrigin = RoomMarker->GetActorLocation();
	
	const float resolution = 1.0 / Quake["accel"]["freq"].asDouble();
	std::vector<FVector> accs;
	for (const auto& v : Quake["accel"]["acc"]) {
		accs.push_back(FVector(v[0].asDouble(), v[1].asDouble(), v[2].asDouble()));
	}

	// Create force gen and start.
	AInertialForceActor* Inertia = GetWorld()->SpawnActor<AInertialForceActor>(RoomOrigin, FRotator(0, 0, 0));
	Inertia->ForceComponent->Radius = 600;
	Inertia->ForceComponent->SetAccelerationProfile(accs, resolution);

	Inertia->ForceComponent->Activate();
	Inertia->ForceComponent->StartPlaying();
	Inertia->ForceComponent->IsConstant = false;

	// Start playing bg sound.
	USoundBase* EqBgSound = LoadObjFromPath<USoundBase>(widen(Quake["bg_sound:asset_full"].asString()).c_str());
	if (!EqBgSound) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("Not playing sound"));
		return;
	}
	UGameplayStatics::PlaySoundAtLocation(GetWorld(), EqBgSound,
		RoomOrigin - FVector(0, 0, -500));
}

void AEqSimGameMode::SpawnInteriorObjects() {
	UWorld* const World = GetWorld();
	if (!World || RuntimeInfo.isNull()) {
		return;
	}

	for (const auto& IObj : RuntimeInfo["interior_objects"]) {
		const FTransform Pose = DeserializeTransform(IObj["pose"]);

		ANoisyActor* Actor = World->SpawnActor<ANoisyActor>(ANoisyActor::StaticClass());
		ActorNames.insert(TCHAR_TO_UTF8(*Actor->GetName()));

		// Dunno why, but you need to set transform first, and then change SM.
		Actor->StaticMeshComponent->SetWorldTransform(Pose);
		Actor->LoadInteriorFullPath(IObj["static_mesh:asset_full"].asString());
		
		GEngine->AddOnScreenDebugMessage(-1, 1, FColor::White, TEXT("+interior object"));
	}
}

void AEqSimGameMode::ResetInteriorObjects() {
	UWorld* const World = GetWorld();
	if (!World || RuntimeInfo.isNull()) {
		return;
	}

	GEngine->AddOnScreenDebugMessage(-1, 1, FColor::White, TEXT("Resetting"));

	// Remove all live actors spawned by SpawnInteriorObjects.
	for (auto It = TActorIterator<ANoisyActor>(World); It; ++It) {
		const std::string name = TCHAR_TO_UTF8(*It->GetName());
		if (ActorNames.find(name) == ActorNames.end()) {
			continue;
		}
		It->Destroy();
	}
	ActorNames.clear();

	SpawnInteriorObjects();
}

FTransform AEqSimGameMode::DeserializeTransform(const Json::Value& Trans) {
	return FTransform(
		FQuat(Trans["quat"]["x"].asDouble(), Trans["quat"]["y"].asDouble(),
		Trans["quat"]["z"].asDouble(), Trans["quat"]["w"].asDouble()),
		FVector(Trans["pos"]["x"].asDouble(), Trans["pos"]["y"].asDouble(), Trans["pos"]["z"].asDouble()));
}

AActor* AEqSimGameMode::FindTargetPointByName(const std::string& name) {
	UWorld* const World = GetWorld();
	if (!World) {
		return nullptr;
	}

	TArray<AActor*> Targets;
	UGameplayStatics::GetAllActorsOfClass(World, ATargetPoint::StaticClass(), Targets);
	for (AActor* Target : Targets) {
		if (TCHAR_TO_UTF8(*Target->GetName()) == name) {
			return Target;
		}
	}
	return nullptr;
}
