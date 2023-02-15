// Fill out your copyright notice in the Description page of Project Settings.


#include "PhysicsObjectCMC.h"

#include "Kismet/KismetMathLibrary.h"

UPhysicsObjectCMC::UPhysicsObjectCMC()
{
}

void UPhysicsObjectCMC::PhysCustom(float deltaTime, int32 Iterations)
{
	Super::PhysCustom(deltaTime, Iterations);
	PhysMoveObject(deltaTime, Iterations);
}

void UPhysicsObjectCMC::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Warning, TEXT("BeginPlay() wooo."));
}

void UPhysicsObjectCMC::PhysMoveObject(float deltaTime, int32 Iterations)
{
	if (deltaTime < MIN_TICK_TIME)
	{
		return;
	}

	// UE_LOG(LogTemp, Warning, TEXT("Phys is ticking."));
	
	RestorePreAdditiveRootMotionVelocity();

	// Apply Gravity
	Velocity = ApplyGravityVector(Velocity, deltaTime, Iterations);

	// Apply Drag
	Velocity = CalculateAirResistanceVector(Velocity, deltaTime);

	float CurrentFriction = FallingLateralFriction;
	
	// Calc Velocity
	if (!HasAnimRootMotion() && !CurrentRootMotion.HasOverrideVelocity())
	{
		CalcVelocity(deltaTime, CurrentFriction, false, GetMaxBrakingDeceleration());
	}
	ApplyRootMotionToVelocity(deltaTime);

	// Perform Move
	Iterations++;
	bJustTeleported = false;
	
	FVector OldLocation = UpdatedComponent->GetComponentLocation();
	FVector Adjusted = Velocity * deltaTime;
	FHitResult Hit(1.f);
	SafeMoveUpdatedComponent(Adjusted, UpdatedComponent->GetComponentQuat(), true, Hit);

	bool bSkipThis = false;
	if (!bSkipThis && Hit.Time < 1.f)
	{

		const FVector GravDir = FVector(0.f, 0.f, -1.f);
		const FVector VelDir = Velocity.GetSafeNormal();

		// Apply friction
		Velocity = ApplyFriction(Velocity, (1.f - Hit.Time));
		
		const float UpDown = GravDir | VelDir;
		bool bSteppedUp = false;
		if ((FMath::Abs(Hit.ImpactNormal.Z) < 0.2f) && (UpDown < 0.5f) && (UpDown > -0.2f) && CanStepUp(Hit))
		{
			float stepZ = UpdatedComponent->GetComponentLocation().Z;
			bSteppedUp = StepUp(GravDir, Adjusted * (1.f - Hit.Time), Hit);
			if (bSteppedUp)
			{
				OldLocation.Z = UpdatedComponent->GetComponentLocation().Z + (OldLocation.Z - stepZ);
			}
		}

		if (!bSteppedUp)
		{
			//adjust and try again
			HandleImpact(Hit, deltaTime, Adjusted);
			
			bool bShouldSlide = BounceSurface(Adjusted, (1.f - Hit.Time), Hit.Normal, Hit, true) <= 1.f;
			// bool bShouldSlide = false;
			if (bShouldSlide)
			{
				SlideAlongSurface(Adjusted, (1.f - Hit.Time), Hit.Normal, Hit, true);
			}
		}
	}

	// Update Outgoing Velocity & Acceleration
	if (!bJustTeleported && !HasAnimRootMotion() && !CurrentRootMotion.HasOverrideVelocity())
	{
		Velocity = (UpdatedComponent->GetComponentLocation() - OldLocation) / deltaTime; // v = dx / dt
	}
}

// bool bAddFriction = false;
// // Add friction if touching floor... maybe change to all types of hits
// const FVector PawnLocation = UpdatedComponent->GetComponentLocation();
// FFindFloorResult FloorResult;
// FindFloor(PawnLocation, FloorResult, false);
// if (bAddFriction && FloorResult.IsWalkableFloor() && IsValidLandingSpot(PawnLocation, FloorResult.HitResult))
// {
// 	CurrentFriction = 1.05f;
// }
// else
// {
// 	// Apply gravity
// 	Velocity = NewFallVelocity(Velocity, Gravity, timeTick);
// }

float UPhysicsObjectCMC::BounceSurface(const FVector& Delta, float Time, const FVector& Normal, FHitResult& Hit, bool bHandleImpact)
{
	if (!Hit.bBlockingHit)
	{
		return 0.f;
	}

	float PercentTimeApplied = 0.f;
	const FVector OldHitNormal = Normal;

	FVector BounceDelta = ComputeBounceDelta(Delta, Time, Normal, Hit);
	float ImpulseSize = BounceDelta.ProjectOnTo(Normal).Size();
	ImpulseSize = 1.f;
	if (ImpulseSize > 0.1f)
	{
		UE_LOG(LogTemp, Warning, TEXT("ImpulseSize: %f"), ImpulseSize);
		UE_LOG(LogTemp, Warning, TEXT("Bounce"));
		const FQuat Rotation = UpdatedComponent->GetComponentQuat();
		SafeMoveUpdatedComponent(BounceDelta, Rotation, true, Hit);

		const float FirstHitPercent = Hit.Time;
		PercentTimeApplied = FirstHitPercent;
		if (Hit.IsValidBlockingHit())
		{
			// Notify first impact
			if (bHandleImpact)
			{
				HandleImpact(Hit, FirstHitPercent * Time, BounceDelta);
			}
		}
	}

	return ImpulseSize;
}

FVector UPhysicsObjectCMC::ComputeBounceDelta(const FVector& Delta, float Time, const FVector& Normal, const FHitResult& Hit) const
{
	float RestituteForce = 0.5f;
	FVector ImpulseFromNormal = Delta.ProjectOnToNormal(Hit.Normal);
	FVector ParallelForce = (Delta - ImpulseFromNormal);
	FVector BounceDelta = (ParallelForce - (RestituteForce * ImpulseFromNormal)); // * Time;
	
	// UE_LOG(LogTemp, Warning, TEXT("BounceDelta: %f"), BounceDelta.Size());
	return BounceDelta;
}

FVector UPhysicsObjectCMC::ApplyGravityVector(const FVector& Delta, float DeltaTime, int32 Iterations)
{
	// Time Acceleration Gravity
	float timeTick = GetSimulationTimeStep(DeltaTime, Iterations);
	// Compute current gravity
	const FVector Gravity(0.f, 0.f, GetGravityZ());
	const FVector NewVelocity = NewFallVelocity(Delta, Gravity, timeTick);
	return NewVelocity;
}

FVector UPhysicsObjectCMC::CalculateAirResistanceVector(const FVector& InitialVelocity, float DeltaTime)
{
	FVector Result = InitialVelocity;
	FVector AirResistanceVector = FVector(0.0f, 0.0f, 0.0f);

	if (DeltaTime > 0)
	{
		FVector DirectionVector = InitialVelocity;
		float V = DirectionVector.Size();
		UKismetMathLibrary::Vector_Normalize(DirectionVector, 0.000100f);
		DirectionVector = UKismetMathLibrary::Multiply_VectorFloat(DirectionVector, -1.0f);
		constexpr float Radius = 0.2;
		constexpr float P = 1.225f;
		constexpr float CD = 0.47f;
		constexpr float Area = PI * Radius * Radius;
		float ForceAmount =  (P * V * V * CD * Area) / 2;
		constexpr float Strength = .009f; // Reduce Air Resistance to ~realistic proportions.
		ForceAmount *= Strength;
		ForceAmount *= DeltaTime;
		AirResistanceVector = UKismetMathLibrary::Multiply_VectorFloat(DirectionVector, ForceAmount);
	}
	Result = UKismetMathLibrary::Add_VectorVector(Result, AirResistanceVector);
	return Result;
}

FVector UPhysicsObjectCMC::ApplyFriction(const FVector& Delta, float DeltaTime)
{
	// // Add friction
	// const float Friction = 1.3f;
	// const float CurrentSpeedSq = Adjusted.SizeSquared();
	// const float VelSize = FMath::Sqrt(CurrentSpeedSq);
	//
	// // UE_LOG(LogTemp, Warning, TEXT("VelSize: %f."), VelSize);
	//
	// FVector FrictionVector = (VelDir * VelSize) * FMath::Min((1.f - Hit.Time) * 0.125f, 1.f);
	//
	// // UE_LOG(LogTemp, Warning, TEXT("Adjusted Vector: %f, Friction Vector: %f."), Adjusted.Size(), FrictionVector.Size());
	//
	// Adjusted = Adjusted - FrictionVector;
	return Delta;
}
