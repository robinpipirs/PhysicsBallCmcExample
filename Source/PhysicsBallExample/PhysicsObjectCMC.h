// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PhysicsBallExampleCharacter.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "PhysicsObjectCMC.generated.h"

/**
 * 
 */
UCLASS()
class PHYSICSBALLEXAMPLE_API UPhysicsObjectCMC : public UCharacterMovementComponent
{
	GENERATED_BODY()
public:
	UPhysicsObjectCMC();

	UPROPERTY(Transient) APhysicsBallExampleCharacter* PhysCharacterOwner;

protected:
	virtual void PhysCustom(float deltaTime, int32 Iterations) override;
	virtual void BeginPlay() override;
private:
	void PhysMoveObject(float deltaTime, int32 Iterations);
	float BounceSurface(const FVector& Delta, float Time, const FVector& Normal, FHitResult& Hit, bool bHandleImpact);
	FVector ComputeBounceDelta(const FVector& Delta, float Time, const FVector& Normal, const FHitResult& Hit) const;
	FVector ApplyGravityVector(const FVector& Delta, float DeltaTime, int32 Iterations);
	FVector CalculateAirResistanceVector(const FVector& InitialVelocity, float DeltaTime);
};
