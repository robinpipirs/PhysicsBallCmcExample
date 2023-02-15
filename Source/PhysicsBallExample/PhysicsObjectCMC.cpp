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

	// Maybe this is missing.. calculating iterations etc
	// while (bSimulationEnabled && RemainingTime >= MIN_TICK_TIME && (Iterations < MaxSimulationIterations) && IsValid(ActorOwner) && !HasStoppedSimulation())
	// {

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
			
			bool bBounced = BounceSurface(Adjusted, (1.f - Hit.Time), Hit.Normal, Hit, true);
			// bool bShouldSlide = false;
			if (!bBounced)
			{
				// const float Friction = 0.2f;
				// const FVector Force = Adjusted;
				// const float ForceDotN = (Force | Hit.Normal);
				// if (ForceDotN < 0.f)
				// {
				// 	UE_LOG(LogTemp, Warning, TEXT("ForceDotN()"));
				// 	const FVector ProjectedForce = FVector::VectorPlaneProject(Force, Hit.Normal);
				// 	const FVector NewVelocity = Adjusted + ProjectedForce;
				//
				// 	const FVector FrictionForce = -NewVelocity.GetSafeNormal() * FMath::Min(-ForceDotN * Friction, NewVelocity.Size());
				// 	Adjusted = ConstrainDirectionToPlane(NewVelocity + FrictionForce);
				// }

				const float RequestedSpeedSquared = Adjusted.SizeSquared();
				if (!(RequestedSpeedSquared < UE_KINDA_SMALL_NUMBER))
				{
					float Friction = 0.3f;
					Friction = FMath::Max(0.f, Friction);
					const float MaxAccel = GetMaxAcceleration();
					float MaxSpeed = GetMaxSpeed();
					
					float RequestedSpeed = FMath::Sqrt(RequestedSpeedSquared);
					const FVector RequestedMoveDir = Adjusted / RequestedSpeed;
					RequestedSpeed = FMath::Min(MaxSpeed, RequestedSpeed);
			
					// Compute actual requested velocity
					const float CurrentSpeedSq = Adjusted.SizeSquared();
					
					const float VelSize = FMath::Sqrt(CurrentSpeedSq);
					UE_LOG(LogTemp, Warning, TEXT("VelSize: %f."),VelSize);
					const float DeltaTime = (1.f - Hit.Time);

					FVector Multiplier = Adjusted - (RequestedMoveDir * VelSize * (0.9f));
					UE_LOG(LogTemp, Warning, TEXT("Multiplier: (%f)."), Multiplier.Size());
					FVector FrictionVector = (Multiplier) * FMath::Min(   DeltaTime * Friction, 1.f);
					UE_LOG(LogTemp, Warning, TEXT("Adjusted - FrictionVector: (%f - %f)."), Adjusted.Size(), FrictionVector.Size());
					Adjusted = Adjusted - FrictionVector;

					UE_LOG(LogTemp, Warning, TEXT("SlideAlongSurface()"));
					SlideAlongSurface(Adjusted, (1.f - Hit.Time), Hit.Normal, Hit, true);
				}
			}
		}
	}

	// Update Outgoing Velocity & Acceleration
	if (!bJustTeleported && !HasAnimRootMotion() && !CurrentRootMotion.HasOverrideVelocity())
	{
		Velocity = (UpdatedComponent->GetComponentLocation() - OldLocation) / deltaTime; // v = dx / dt
	}
}

bool UPhysicsObjectCMC::BounceSurface(const FVector& Delta, float Time, const FVector& Normal, FHitResult& Hit, bool bHandleImpact)
{
	if (!Hit.bBlockingHit)
	{
		return 0.f;
	}

	float PercentTimeApplied = 0.f;
	const FVector OldHitNormal = Normal;
	float BounceVelocityStopSimulatingThreshold = 5.f;


	FVector BounceDelta = ComputeBounceDelta(Delta, Time, Normal, Hit);
	if (!(BounceDelta.SizeSquared() < FMath::Square(BounceVelocityStopSimulatingThreshold)))
	{
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
		return true;
	}

	return false;
}

/**
 * Controls the effects of friction on velocity parallel to the impact surface when bouncing.
 * If true, friction will be modified based on the angle of impact, making friction higher for perpendicular impacts and lower for glancing impacts.
 * If false, a bounce will retain a proportion of tangential velocity equal to (1.0 - Friction), acting as a "horizontal restitution".
 */
// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=ProjectileBounces)
// uint8 bBounceAngleAffectsFriction:1;

FVector UPhysicsObjectCMC::ComputeBounceDelta(const FVector& Delta, float Time, const FVector& HitNormal, const FHitResult& Hit) const
{
	float Bounciness = 0.6f;
	float Friction = 0.2f;
	float MinFrictionFraction = 0.0f;
	bool bIsSliding = false;

	FVector TempVelocity = Delta;
	const FVector Normal = ConstrainNormalToPlane(Hit.Normal);
	const float VDotNormal = (Delta | Normal);

	// Only if velocity is opposed by normal or parallel
	if (VDotNormal <= 0.f)
	{
		// Project velocity onto normal in reflected direction.
		const FVector ProjectedNormal = Normal * -VDotNormal;

		// Point velocity in direction parallel to surface
		TempVelocity += ProjectedNormal;

		// Only tangential velocity should be affected by friction.
		const float ScaledFriction = (true || bIsSliding) ? FMath::Clamp(-VDotNormal / TempVelocity.Size(), MinFrictionFraction, 1.f) * Friction : Friction;
		TempVelocity *= FMath::Clamp(1.f - ScaledFriction, 0.f, 1.f);

		// Coefficient of restitution only applies perpendicular to impact.
		TempVelocity += (ProjectedNormal * FMath::Max(Bounciness, 0.f));

		// Bounciness could cause us to exceed max speed.
		TempVelocity = LimitVelocity(TempVelocity);
	}
	
	// UE_LOG(LogTemp, Warning, TEXT("BounceDelta: %f"), BounceDelta.Size());
	return TempVelocity;
}

FVector UPhysicsObjectCMC::LimitVelocity(FVector NewVelocity) const
{
	const float CurrentMaxSpeed = GetMaxSpeed();
	if (CurrentMaxSpeed > 0.f)
	{
		NewVelocity = NewVelocity.GetClampedToMaxSize(CurrentMaxSpeed);
	}

	return ConstrainDirectionToPlane(NewVelocity);
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
