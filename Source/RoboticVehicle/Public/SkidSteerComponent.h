// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "MotorSystem.h"
#include "CoreMinimal.h"
#include "SimpleElectricVehicle.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "SkidSteerComponent.generated.h"

USTRUCT()
struct ROBOTICVEHICLE_API FVehicleMotorConfig
{
    GENERATED_USTRUCT_BODY();
    FVehicleMotorConfig()
    {
        InitDefaults();
    }

    /** Armature Resistance (Ohms) */
    UPROPERTY(EditAnywhere, Category = Setup)
    float ArmatureResistance;

    /** Motor Max Voltage (Volts) */
    UPROPERTY(EditAnywhere, Category = Setup)
    float MotorVoltage;

    /** Torque Constant (Nm/Amp)*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float TorqueConstant;

    /** EMF Constant (V/RPM)*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float EMFConstant;

    /** Gear Ratio (unitless)*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float OverallRatio;

    /** Wheel System Rotational Moment of Inertia (kg*m^2)*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float MotorSystemMOI;

    /** Drive System Friction at Motor (Nm)*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float MotorSystemFriction;

    /** Proportional Gain for Speed Control */
    UPROPERTY(EditAnywhere, Category = Setup)
    float ProportionalGain;

    /** Integral Gain for Speed Control */
    UPROPERTY(EditAnywhere, Category = Setup)
    float IntegralGain;

    /** Derivative Gain for Speed Control */
    UPROPERTY(EditAnywhere, Category = Setup)
    float DerivativeGain;

    void InitDefaults()
    {
        ArmatureResistance = 0.091;
		MotorVoltage = 12;
        TorqueConstant = .0188;
        EMFConstant = .0188;
		OverallRatio = 1;
		MotorSystemMOI = 1;
		MotorSystemFriction = 0;
        ProportionalGain = 0.5;
        IntegralGain = 0;
        DerivativeGain = 0;
    }

	const FSimpleMotorConfig& GetPhysicsMotorConfig()
	{
		FillMotorSetup();
		return PMotorConfig;
	}


private:

	void FillMotorSetup()
	{
		PMotorConfig.ArmatureResistance = this->ArmatureResistance;
		PMotorConfig.MotorVoltage = this->MotorVoltage;
		PMotorConfig.TorqueConstant = this->TorqueConstant;
		PMotorConfig.EMFConstant = this->EMFConstant;
		PMotorConfig.OverallRatio = this->OverallRatio;
		PMotorConfig.MotorSystemMOI = this->MotorSystemMOI;
		PMotorConfig.MotorSystemFriction = this->MotorSystemFriction;
        PMotorConfig.ProportionalGain = this->ProportionalGain;
        PMotorConfig.IntegralGain = this->IntegralGain;
        PMotorConfig.DerivativeGain = this->DerivativeGain;
	}

	FSimpleMotorConfig PMotorConfig;
};

class ROBOTICVEHICLE_API USkidSteerSimulation : public UChaosWheeledVehicleSimulation
{
public:
	void UpdateSimulation(float DeltaTime, const FChaosVehicleDefaultAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override;
	virtual void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) override;
	virtual void ProcessElectroMechanicalSimulation(float DeltaTime);
    virtual void ApplyWheelFrictionForces(float DeltaTime) override;

	
	virtual ~USkidSteerSimulation()
	{
	}

    virtual void SetIndividualWheelControl(bool _IndividualWheelControl)
    {
        IndividualWheelControl = _IndividualWheelControl;
    }

    virtual int GetNumSpeeds()
    {
        return desiredSpeeds.Num();
    }

    virtual void SetTargetWheelSpeed(float RPM, int WheelIdx)
    {
        desiredSpeeds[WheelIdx] = RPM;
    }

    virtual void SetNumMotors(int NumMotors)
    {
        for(int MotorIdx = 0; MotorIdx < NumMotors; MotorIdx++)
        {
            desiredSpeeds.Add(0);
        }
    }

    TArray<float> desiredSpeeds;
    // Flag to Enable Individual Wheel Control
    bool IndividualWheelControl;

protected:

private:

};

/**
 * 
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class ROBOTICVEHICLE_API USkidSteerComponent : public UChaosWheeledVehicleMovementComponent
{
	GENERATED_UCLASS_BODY()

public:
	/** Motors */
	UPROPERTY(EditAnywhere, Category = MotorSetup)
	TArray<FVehicleMotorConfig> MotorSetups;

    /** Enable Individual Wheel Speed Control (Disables Throttle Control) */
    UPROPERTY(EditAnywhere, Category = MotorSetup)
    bool IndividualWheelControl;

    UFUNCTION(BlueprintCallable, Category = "Game|Components|ElectricVehicleMovement|")
    void SetTargetWheelSpeed(float RPM, int WheelIdx);

    UFUNCTION(BlueprintCallable, Category = "Game|Components|ElectricVehicleMovement|")
    float GetWheelSpeed(int WheelIdx);
protected:
	virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() override
	{
		// Make the Vehicle Simulation class that will be updated from the physics thread async callback
		VehicleSimulationPT = MakeUnique<USkidSteerSimulation>();
        ElectricVehicleSimulationPT = static_cast<USkidSteerSimulation*>(VehicleSimulationPT.Get());
		PVehicleOutput = MakeUnique<FPhysicsVehicleOutput>();
		return MakeUnique<FSimpleElectricWheeledVehicle>();
	}

    virtual void ProcessSleeping(const FControlInputs& ControlInputs) override 
    {
        SetSleeping(false);
    }

	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) override;
    virtual void ParallelUpdate(float DeltaSeconds);

    virtual void FillWheelOutputSpeeds();

    USkidSteerSimulation* ElectricVehicleSimulationPT;
	TArray<FSimpleMotorSim> Motors;
    TArray<float> outputSpeeds;
	
};
