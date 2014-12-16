

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



	static ConstructorHelpers::FObjectFinder<USoundWave> Sound(TEXT("/Game/Audio/EarthquakeBG.EarthquakeBG"));
	EqBgSound = Sound.Object;
	if (!EqBgSound) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("BG sound load failed"));
	}
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
	
#if 0
	// Accel gen.
	{
		// Load acc profile.
		picojson::value acc_root;
		{
			std::ifstream fs_acc("C:\\VR14a\\Hachi.json");
			if (!fs_acc.is_open()) {
				GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("Hachi.json not found!!! Aborting experiment."));
				return;
			}
			GEngine->AddOnScreenDebugMessage(-1, 1, FColor::White, TEXT("Opened acc prof"));
			fs_acc >> acc_root;
		}

		// cm/s^2 == uu/s^2
		if (!acc_root.is<picojson::object>() || !acc_root.get<picojson::object>()["freq"].is<double>()) {
			GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("Hachi.json: wrong format"));
			return;
		}
		const float resolution = 1.0 / acc_root.get<picojson::object>()["freq"].get<double>();
		std::vector<FVector> accs;
		for (auto& v : acc_root.get<picojson::object>()["accel"].get<picojson::array>()) {
			auto va = v.get<picojson::array>();
			accs.push_back(FVector(va[0].get<double>(), va[1].get<double>(), va[2].get<double>()) * 10);
		}
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Green,
			FString::Printf(TEXT("Loaded %d entries w/ delta=%f s"), accs.size(), resolution));

		// Create force gen.
		AInertialForceActor* Inertia = World->SpawnActor<AInertialForceActor>(AInertialForceActor::StaticClass());
		Inertia->ForceComponent->Radius = 1500;
		Inertia->ForceComponent->SetAccelerationProfile(accs, resolution);

		Inertia->ForceComponent->Activate();
		Inertia->ForceComponent->StartPlaying();
		Inertia->ForceComponent->IsConstant = false;
	}
	

	// Play BG sound
	if (EqBgSound) {
		UGameplayStatics::PlaySoundAtLocation(World, EqBgSound, FVector(0, 0, -1000));
	}
#endif
}

void AEqSimGameMode::PrepareSubExperiment(int SubExperimentId) {

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

	// Remove all live actors spawned by SpawnInteriorObjects.
	for (auto It = TActorIterator<ANoisyActor>(World); It; ++It) {
		const std::string name = TCHAR_TO_UTF8(*It->GetName());
		if (ActorNames.find(name) == ActorNames.end()) {
			continue;
		}
		World->RemoveActor(static_cast<AActor*>(*It), false);
	}
	ActorNames.clear();

}

FTransform AEqSimGameMode::DeserializeTransform(const Json::Value& Trans) {
	return FTransform(
		FQuat(Trans["quat"]["x"].asDouble(), Trans["quat"]["y"].asDouble(),
		Trans["quat"]["z"].asDouble(), Trans["quat"]["w"].asDouble()),
		FVector(Trans["pos"]["x"].asDouble(), Trans["pos"]["y"].asDouble(), Trans["pos"]["z"].asDouble()));
}
