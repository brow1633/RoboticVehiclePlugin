// Fill out your copyright notice in the Description page of Project Settings.


#include "SkidSteerComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"

#include "DrawDebugHelpers.h"
#include "DisplayDebugHelpers.h"
#include "DisplayDebugHelpers.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "VehicleAnimationInstance.h"
#include "ChaosVehicleManager.h"
#include "ChaosVehicleWheel.h"
#include "SuspensionUtility.h"
#include "SteeringUtility.h"
#include "TransmissionUtility.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Chaos/DebugDrawQueue.h"
#include "UObject/UE5MainStreamObjectVersion.h"

#include "PhysicsProxy/SuspensionConstraintProxy.h"
#include "PBDRigidsSolver.h"

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#endif
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

using namespace Chaos;

USkidSteerComponent::USkidSteerComponent(const FObjectInitializer &ObjectInitializer) :
    Super(ObjectInitializer)
{
	for(int32 MotorIdx = 0; MotorIdx < MotorSetups.Num(); ++MotorIdx)
	{
		MotorSetups[MotorIdx].InitDefaults();
	}
}

void USkidSteerComponent::SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle> &PVehicle)
{
	check(PVehicle);

	Super::SetupVehicle(PVehicle);
    check(VehicleSimulationPT);
    ElectricVehicleSimulationPT->SetIndividualWheelControl(IndividualWheelControl);

	FSimpleElectricWheeledVehicle* PElectricVehicle (static_cast<FSimpleElectricWheeledVehicle*>(PVehicle.Get()));

	check(PElectricVehicle);

	for(int32 MotorIdx = 0; MotorIdx < MotorSetups.Num(); ++MotorIdx)
	{
		outputSpeeds.Add(0.f);
		FSimpleMotorSim MotorSim(&MotorSetups[MotorIdx].GetPhysicsMotorConfig());
		PElectricVehicle->Motors.Add(MotorSim);
	}
	ensure(outputSpeeds.Num() == MotorSetups.Num());

    ElectricVehicleSimulationPT->SetNumMotors(MotorSetups.Num());
}

void USkidSteerComponent::ParallelUpdate(float DeltaSeconds)
{
    FillWheelOutputSpeeds();
    UChaosWheeledVehicleMovementComponent::ParallelUpdate(DeltaSeconds);
}

void USkidSteerComponent::FillWheelOutputSpeeds()
{
    for(int WheelIdx = 0; WheelIdx < outputSpeeds.Num(); WheelIdx++)
    {
        auto& PWheel = PVehicleOutput->Wheels[WheelIdx];
        outputSpeeds[WheelIdx] = Chaos::OmegaToRPM(PWheel.AngularVelocity);
    }
}

float USkidSteerComponent::GetWheelSpeed(int WheelIdx)
{
    if(WheelIdx < outputSpeeds.Num())
    {
        return outputSpeeds[WheelIdx];
    }
    return 0.f;
}

void USkidSteerComponent::SetTargetWheelSpeed(float RPM, int WheelIdx)
{
    if(WheelIdx < ElectricVehicleSimulationPT->GetNumSpeeds())
    {
        ElectricVehicleSimulationPT->SetTargetWheelSpeed(RPM, WheelIdx);
	    UE_LOG(LogTemp, Warning, TEXT("Setting from BluePrint %lf"), RPM)
    }
}

void USkidSteerSimulation::UpdateSimulation(float DeltaTime, const FChaosVehicleDefaultAsyncInput &InputData, Chaos::FRigidBodyHandle_Internal *Handle)
{
	FSimpleElectricWheeledVehicle* PElectricVehicle (static_cast<FSimpleElectricWheeledVehicle*>(PVehicle.Get()));
	check(PElectricVehicle);

	if (CanSimulate() && Handle)
	{
		ensure(PElectricVehicle->Wheels.Num() == PElectricVehicle->Motors.Num());
	}

	ProcessElectroMechanicalSimulation(DeltaTime);

	UChaosWheeledVehicleSimulation::UpdateSimulation(DeltaTime, InputData, Handle);
}

void USkidSteerSimulation::ApplyInput(const FControlInputs &ControlInputs, float DeltaTime)
{
	UChaosVehicleSimulation::ApplyInput(ControlInputs, DeltaTime);

	FControlInputs ModifiedInputs = ControlInputs;
	FSimpleElectricWheeledVehicle* PElectricVehicle (static_cast<FSimpleElectricWheeledVehicle*>(PVehicle.Get()));
	check(PElectricVehicle);

	if(PElectricVehicle->HasMotor()) 
	{
		for(int MotorIdx = 0; MotorIdx < PElectricVehicle->Motors.Num(); MotorIdx++)
		{
			auto& PMotor = PElectricVehicle->Motors[MotorIdx];
            if(IndividualWheelControl)
            {
                UE_LOG(LogTemp, Warning, TEXT("Applying Input with Desired Speed: %lf"), desiredSpeeds[MotorIdx]);
                ensure(MotorIdx < desiredSpeeds.Num());
                PMotor.SetTargetWheelSpeed(desiredSpeeds[MotorIdx]);
            }
            else
            {
                PMotor.SetThrottle(ModifiedInputs.ThrottleInput-ModifiedInputs.BrakeInput);
            }
			/*UE_LOG(LogTemp, Warning, TEXT("Input: %lf, Brake Input: %lf"), 	ModifiedInputs.ThrottleInput,
																			ModifiedInputs.BrakeInput);*/
		}
	}
}

void USkidSteerSimulation::ProcessElectroMechanicalSimulation(float DeltaTime)
{
	FSimpleElectricWheeledVehicle* PElectricVehicle (static_cast<FSimpleElectricWheeledVehicle*>(PVehicle.Get()));
	check(PElectricVehicle);

	if(PElectricVehicle->Motors.Num() == PElectricVehicle->Wheels.Num()) {
		for(int WheelIdx = 0; WheelIdx < PElectricVehicle->Motors.Num(); ++WheelIdx)
		{
			auto& PMotor = PElectricVehicle->Motors[WheelIdx];
			auto& PWheel = PElectricVehicle->Wheels[WheelIdx];

			float WheelRPM = PWheel.GetWheelRPM();
			PMotor.SetMotorRPM(WheelRPM);
			PMotor.Simulate(DeltaTime);
			float MotorTorque = PMotor.GetWheelTorque();

			PWheel.SetDriveTorque(Chaos::TorqueMToCm(MotorTorque));
			UE_LOG(LogTemp, Warning, TEXT("Drive Torque: %lf, Wheel Speed: %lf"), MotorTorque, PWheel.GetWheelRPM());
		}
	}
}

void USkidSteerSimulation::ApplyWheelFrictionForces(float DeltaTime)
{
	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
	{
		auto& PWheel = PVehicle->Wheels[WheelIdx]; // Physics Wheel
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		if (HitResult.PhysMaterial.IsValid())
		{
			PWheel.SetSurfaceFriction(HitResult.PhysMaterial->Friction);
		}

		// take into account steering angle
		float SteerAngleDegrees = PWheel.SteeringAngle;
		FRotator SteeringRotator(0.f, SteerAngleDegrees, 0.f);
		FVector SteerLocalWheelVelocity = SteeringRotator.UnrotateVector(WheelState.LocalWheelVelocity[WheelIdx]);

		PWheel.SetVehicleGroundSpeed(SteerLocalWheelVelocity);
		PWheel.Simulate(DeltaTime);

		if (PWheel.InContact())
		{
			float RotationAngle = FMath::RadiansToDegrees(PWheel.GetAngularPosition());
			FVector FrictionForceLocal = PWheel.GetForceFromFriction();
			FrictionForceLocal = SteeringRotator.RotateVector(FrictionForceLocal);

			FVector GroundZVector = HitResult.Normal;
			FVector GroundXVector = FVector::CrossProduct(VehicleState.VehicleRightAxis, GroundZVector);
			FVector GroundYVector = FVector::CrossProduct(GroundZVector, GroundXVector);
			
			FMatrix Mat = FMatrix(GroundXVector, GroundYVector, GroundZVector, VehicleState.VehicleWorldTransform.GetLocation());
			FVector FrictionForceVector = Mat.TransformVector(FrictionForceLocal);

			check(PWheel.InContact());
			if (PVehicle->bLegacyWheelFrictionPosition)
			{
				AddForceAtPosition(FrictionForceVector, WheelState.WheelWorldLocation[WheelIdx]);
			}
			else
			{
				AddForceAtPosition(FrictionForceVector, HitResult.ImpactPoint);
			}

		}
		else
		{
			PWheel.SetVehicleGroundSpeed(FVector::ZeroVector);
			PWheel.SetWheelLoadForce(0.f);
			PWheel.Simulate(DeltaTime);
		}

	}
}