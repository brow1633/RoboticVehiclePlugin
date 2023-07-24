#pragma once

#include "Math/UnrealMathSSE.h"
#include "VehicleSystemTemplate.h"
#include "VehicleUtility.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

struct ROBOTICVEHICLE_API FSimpleMotorConfig
{
    FSimpleMotorConfig()
        : ArmatureResistance(0.091)
        , MotorVoltage(12)
        , TorqueConstant(.0188)
        , EMFConstant(.0188)
		, OverallRatio(1)
		, MotorSystemMOI(1)
        , MotorSystemFriction(0)
        , ProportionalGain(0.5)
        , IntegralGain(0.f)
        , DerivativeGain(0.f)
        {

        }

        float ArmatureResistance;
        float MotorVoltage;
        float TorqueConstant;
        float EMFConstant;
		float OverallRatio;
		float MotorSystemMOI;
		float MotorSystemFriction;
        float ProportionalGain;
        float IntegralGain;
        float DerivativeGain;
};

class ROBOTICVEHICLE_API FSimpleMotorSim : public TVehicleSystem<FSimpleMotorConfig>
{
public:

    FSimpleMotorSim(const FSimpleMotorConfig* StaticDataIn);

    void SetThrottle(float InThrottle)
    {
        FreeRunning = false;
        InThrottle = FMath::Clamp(InThrottle, -1.f, 1.f);
        Throttle = InThrottle;
        if(FMath::Abs(InThrottle) < SMALL_NUMBER)
        {
            InThrottle = 0;
            FreeRunning = true;
        }
    }

    void SetTargetWheelSpeed(float RPM)
    {
        TargetSpeed = RPM * Setup().OverallRatio;
    }

    // Takes in desired wheel velocity (since we have transmission)
    void SetMotorRPM(float InWheelRPM)
    {
        if(!FreeRunning)
        {
            GroundSpeed = InWheelRPM * Setup().OverallRatio;
        }
    }

    float GetMotorTorque()
    {
        return GetTorqueFromVoltage(Throttle*Setup().MotorVoltage);
    }

    float GetWheelTorque()
    {
        return GetMotorTorque() * Setup().OverallRatio;
    }

    float GetTorqueFromVoltage(float MotorVoltageIn)
    {
        return GetTorqueFromVoltage(MotorVoltageIn, CurrentRPM);
    }

    /* Get torque value based on input RPM and Voltage */
    float GetTorqueFromVoltage(float MotorVoltageIn, float RPM);

    /* Get motor speed in Revolutions per Minute */
    float GetMotorRPM() const
    {
        return CurrentRPM;
    }

    void Simulate(float DeltaTime);

protected:
    float Throttle; // [-1,1] normalized throttle
    float GroundSpeed;  // Wheel's Ground Speed
    float CurrentRPM;   // current RPM
    float TargetSpeed; // target wheel speed [RPM]
    float DriveTorque;  // current torque [N.m]

    float IntegralAcc; // Accumulated Integral Term
    float PrevError;   // prev error for derivative term

    bool FreeRunning;   // is motor in neutral with no load

    float Omega;        // motor speed

    
};
