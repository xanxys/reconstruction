

#include "EqExperiment.h"
#include "EqSimGameMode.h"

#include <fstream>
#include <string>

#include "picojson.h"
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
	UWorld* const World = GetWorld();
	if (!World) {
		return;
	}

	picojson::value root;
	
	std::ifstream test("C:\\VR14a\\derived_data.json");
	if (!test.is_open()) {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("small_data.json not found; reverting to toy mode"));

		// "toy" mode
		ANoisyActor* Actor0 = World->SpawnActor<ANoisyActor>(ANoisyActor::StaticClass());
		Actor0->LoadInterior("SM_Lamp_Wall");

		ANoisyActor* Actor1 = World->SpawnActor<ANoisyActor>(ANoisyActor::StaticClass());
		Actor1->LoadInterior("SM_Lamp_Ceiling");

		ANoisyActor* Actor2 = World->SpawnActor<ANoisyActor>(ANoisyActor::StaticClass());
		Actor2->LoadInterior("SM_PillarFrame");
		return;
	}
	test >> root;

	if (root.get<picojson::object>()["test"].get<std::string>() == "good") {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Green, TEXT("Ext load ok"));
		auto interior = root.get<picojson::object>()["interior"].get<picojson::array>();
		for (auto& entry : interior) {
			ANoisyActor* Actor = World->SpawnActor<ANoisyActor>(ANoisyActor::StaticClass());
			Actor->LoadInteriorFullPath(entry.get<picojson::object>()["StaticMesh"].get<std::string>());
			GEngine->AddOnScreenDebugMessage(-1, 1, FColor::White, TEXT("+interior object"));
		}
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Green, TEXT("Loaded: ready to go"));
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, TEXT("Ext load failed"));
	}

	// Accel gen.
	{
		AInertialForceActor* Inertia = World->SpawnActor<AInertialForceActor>(AInertialForceActor::StaticClass());
		Inertia->ForceComponent->Radius = 500;
		Inertia->ForceComponent->Activate();
	}
	

	// Play BG sound
	if (EqBgSound) {
		UGameplayStatics::PlaySoundAtLocation(World, EqBgSound, FVector(0, 0, -1000));
	}
}



