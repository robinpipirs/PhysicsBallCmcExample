// Copyright Epic Games, Inc. All Rights Reserved.

#include "PhysicsBallExampleGameMode.h"
#include "PhysicsBallExampleCharacter.h"
#include "UObject/ConstructorHelpers.h"

APhysicsBallExampleGameMode::APhysicsBallExampleGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
