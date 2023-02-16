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
	
	if (Hit.Time < 1.f)
	{
		const FVector GravDir = FVector(0.f, 0.f, -1.f);
		const FVector VelDir = Velocity.GetSafeNormal();

		// Apply friction
		Adjusted = ApplyFriction(Adjusted, (1.f - Hit.Time), Hit);
		
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
			// bool bBounced = false;
			if (!bBounced)
			{
				const float RequestedSpeedSquared = Adjusted.SizeSquared();
				if (!(RequestedSpeedSquared < UE_KINDA_SMALL_NUMBER))
				{
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

FVector UPhysicsObjectCMC::ApplyFriction(const FVector& Delta, float DeltaTime, const FHitResult& Hit)
{
	FVector TempVelocity = Delta;
	const float RequestedSpeedSquared = Delta.SizeSquared();
	if (RequestedSpeedSquared >= UE_KINDA_SMALL_NUMBER)
	{
		float Friction = 0.3f;
		Friction = FMath::Max(0.f, Friction);
		const float MaxAccel = GetMaxAcceleration();
		float MaxSpeed = GetMaxSpeed();
					
		float RequestedSpeed = FMath::Sqrt(RequestedSpeedSquared);
		const FVector RequestedMoveDir = Delta / RequestedSpeed;
		RequestedSpeed = FMath::Min(MaxSpeed, RequestedSpeed);
			
		// Compute actual requested velocity
		const float CurrentSpeedSq = Delta.SizeSquared();
					
		const float VelSize = FMath::Sqrt(CurrentSpeedSq);
		const float DeltaTime = (1.f - Hit.Time);

		FVector Multiplier = Delta - (RequestedMoveDir * VelSize * (0.90f));
		FVector FrictionVector = (Multiplier) * FMath::Min(   DeltaTime * Friction, 1.f);

		// UE_LOG(LogTemp, Warning, TEXT("VelSize: %f."),VelSize);
		// UE_LOG(LogTemp, Warning, TEXT("Multiplier: (%f)."), Multiplier.Size());
		// UE_LOG(LogTemp, Warning, TEXT("Adjusted - FrictionVector: (%f - %f)."), Delta.Size(), FrictionVector.Size());
		TempVelocity = Delta - FrictionVector;
	}
	return TempVelocity;
}

bool UPhysicsObjectCMC::BounceSurface(const FVector& Delta, float Time, const FVector& Normal, FHitResult& Hit, bool bHandleImpact)
{
	if (!Hit.bBlockingHit)
	{
		return 0.f;
	}

	float PercentTimeApplied = 0.f;
	const FVector OldHitNormal = Normal;
	float BounceVelocityStopSimulatingThreshold = 1.f;


	FVector BounceDelta = ComputeBounceDelta(Delta, Time, Normal, Hit);
	const float VBounceDelta = (BounceDelta | Normal);
	if (!(VBounceDelta < FMath::Square(BounceVelocityStopSimulatingThreshold)))
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
	float Friction = 0.4f;
	float MinFrictionFraction = 0.1f;
	bool bBounceAngleAffectsFriction = false;
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
		const float ScaledFriction = bBounceAngleAffectsFriction ? FMath::Clamp(-VDotNormal / TempVelocity.Size(), MinFrictionFraction, 1.f) * Friction : Friction;
		TempVelocity *= FMath::Clamp(1.f - ScaledFriction, 0.f, 1.f);

		// Coefficient of restitution only applies perpendicular to impact.
		TempVelocity += (ProjectedNormal * FMath::Max(Bounciness, 0.f));

		// Bounciness could cause us to exceed max speed.
		TempVelocity = LimitVelocity(TempVelocity);


		UE_LOG(LogTemp, Warning, TEXT("ScaledFriction: %f."), ScaledFriction);
		UE_LOG(LogTemp, Warning, TEXT("Delta: %f."), Delta.SizeSquared());
		UE_LOG(LogTemp, Warning, TEXT("TempVelocity: %f."), TempVelocity.SizeSquared());
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


