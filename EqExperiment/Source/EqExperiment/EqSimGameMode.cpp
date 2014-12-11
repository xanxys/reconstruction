

#include "EqExperiment.h"
#include "EqSimGameMode.h"

#include "NoisyActor.h"


AEqSimGameMode::AEqSimGameMode(const class FPostConstructInitializeProperties& PCIP)
	: Super(PCIP)
{
	DefaultPawnClass = ConstructorHelpers::FClassFinder<APawn>(TEXT("/Game/Blueprints/MyCharacter")).Class;
}

void AEqSimGameMode::BeginPlay() {
	Super::BeginPlay();
	UWorld* const World = GetWorld();
	if (!World) {
		return;
	}

	World->SpawnActor<ANoisyActor>(ANoisyActor::StaticClass());



}



