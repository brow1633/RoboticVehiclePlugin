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

    /** Torque [Noramlized 0..1] for a given RPM */
    UPROPERTY(EditANywhere, Category = Setup)
    FRuntimeFloatCurve TorqueCurve;

    UPROPERTY(EditAnywhere, Category = Setup)
    TSet<int> WheelIndicies;

    /** Max Motor Torque (Nm) is multiplied by TorqueCurve*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float MaxTorque;

    /** Max Motor RPM */
    UPROPERTY(EditAnywhere, Category = Setup)
    float MaxRpm;

    /** Gear Ratio From Wheel to Motor (unitless)*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float OverallRatio;

    /** Motor System Friction At Motor Output Shaft*/
    UPROPERTY(EditAnywhere, Category = Setup)
    float MotorSystemFriction;

    /** Toggle PID Control, allows for SetTargetMotorSpeeds */
    UPROPERTY(EditAnywhere, Category = Setup)
    bool PidControl;

    /** Proportional Gain for Speed Control */
    UPROPERTY(EditAnywhere, Category = Setup, meta = (EditCondition = "PidControl"))
    float ProportionalGain;

    /** Integral Gain for Speed Control */
    UPROPERTY(EditAnywhere, Category = Setup, meta = (EditCondition = "PidControl"))
    float IntegralGain;

    /** Derivative Gain for Speed Control */
    UPROPERTY(EditAnywhere, Category = Setup, meta = (EditCondition = "PidControl"))
    float DerivativeGain;


    void InitDefaults()
    {
		OverallRatio = 1;
        MaxRpm = 1000;
        MaxTorque = 10;
		MotorSystemFriction = 0;
        ProportionalGain = 0.5;
        IntegralGain = 0;
        DerivativeGain = 0;
        PidControl = false;
    }

	const FSimpleMotorConfig& GetPhysicsMotorConfig()
	{
		FillMotorSetup();
		return PMotorConfig;
	}

    float GetTorqueFromRPM(float MotorRPM)
    {
        // The source curve does not need to be normalized, however we are normalizing it when it is passed on,
		// since it's the MaxRpm and MaxTorque values that determine the range of RPM and Torque
		float MinVal = 0.f, MaxVal = 0.f;
		this->TorqueCurve.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
		return TorqueCurve.GetRichCurve()->Eval(MotorRPM) / MaxVal * MaxTorque;
    }


private:

	void FillMotorSetup()
	{
        PMotorConfig.TorqueCurve.Empty();
		float NumSamples = 20;
		for (float X = 0; X <= this->MaxRpm; X+= (this->MaxRpm / NumSamples))
		{ 
			float MinVal = 0.f, MaxVal = 0.f;
			this->TorqueCurve.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
			float Y = this->TorqueCurve.GetRichCurveConst()->Eval(X) / MaxVal;
			PMotorConfig.TorqueCurve.AddNormalized(Y);
		}

        PMotorConfig.WheelIndicies = this->WheelIndicies;
		PMotorConfig.OverallRatio = this->OverallRatio;
        PMotorConfig.MaxRpm = this->MaxRpm;
        PMotorConfig.MaxTorque = this->MaxTorque;
		PMotorConfig.MotorSystemFriction = this->MotorSystemFriction;
        PMotorConfig.ProportionalGain = this->ProportionalGain;
        PMotorConfig.IntegralGain = this->IntegralGain;
        PMotorConfig.DerivativeGain = this->DerivativeGain;
        PMotorConfig.PidControl = this->PidControl;
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

    virtual void SetSpeedControl(bool _SpeedControl)
    {
        SpeedControl = _SpeedControl;
    }

    virtual void SetTargetMotorSpeed(float RPM, int MotorIdx)
    {
        desiredSpeeds[MotorIdx] = RPM;
    }

    virtual void SetMotorThrottle(float Throttle, int MotorIdx)
    {
        desiredThrottles[MotorIdx] = Throttle;
    }

    virtual void SetNumMotors(int NumMotors)
    {
        for(int MotorIdx = 0; MotorIdx < NumMotors; MotorIdx++)
        {
            desiredSpeeds.Add(0);
            desiredThrottles.Add(0);
        }
    }

    virtual int GetNumMotors()
    {
        return desiredSpeeds.Num();
    }

    TArray<float> desiredSpeeds;
    TArray<float> desiredThrottles;

    float StaticLateralFrictionCoefficient;
    float StaticLongitudinalFrictionCoefficient;
    float SkidLateralFrictionCoefficient;
    float SkidLongitudinalFrictionCoefficient;
    float StaticToSkidSlipRatio;

    bool SpeedControl;
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

    /** Enable Speed Control (SetTargetMotorSpeed) instead of throttle control (Must Enable PID in each Motor)*/
    UPROPERTY(EditAnywhere, Category = SkidSteerSetup)
    bool SpeedControl;

    /** Lateral Friction Coefficient in Static (non-skidding) condition*/
    UPROPERTY(EditAnywhere, Category = SkidSteerSetup)
    float StaticLateralFrictionCoefficient;

    /** Longitudinal Friction Coefficient in Static (non-skidding) condition*/
    UPROPERTY(EditAnywhere, Category = SkidSteerSetup)
    float StaticLongitudinalFrictionCoefficient;

    /** Lateral Friction Coefficient in skidding condition*/
    UPROPERTY(EditAnywhere, Category = SkidSteerSetup)
    float SkidLateralFrictionCoefficient;
    
    /** Longitudinal Friction Coefficient in skidding condition*/
    UPROPERTY(EditAnywhere, Category = SkidSteerSetup)
    float SkidLongitudinalFrictionCoefficient;
    
    /** Slip Ratio at which vehicle goes from static to skidding*/
    UPROPERTY(EditAnywhere, Category = SkidSteerSetup)
    float StaticToSkidSlipRatio;

    UFUNCTION(BlueprintCallable, Category = "Game|Components|ElectricVehicleMovement|")
    void SetTargetMotorSpeed(float RPM, int MotorIdx);

    UFUNCTION(BlueprintCallable, Category = "Game|Components|ElectricVehicleMovement|")
    void SetMotorThrottle(float ThrottleValue, int MotorIdx);

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
	//TArray<FSimpleMotorSim> Motors;
    TArray<float> outputSpeeds;
	
};
