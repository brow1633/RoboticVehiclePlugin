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
		: OverallRatio(1)
        , MaxTorque(10)
        , MaxRpm(1000)
        , MotorSystemFriction(0)
        , ProportionalGain(0.5)
        , IntegralGain(0.f)
        , DerivativeGain(0.f)
        , PidControl(false)
        {

        }

        Chaos::FNormalisedGraph TorqueCurve;
        TSet<int> WheelIndicies;
		float OverallRatio;
        float MaxTorque;
        float MaxRpm;
		float MotorSystemFriction;
        float ProportionalGain;
        float IntegralGain;
        float DerivativeGain;
        bool PidControl;
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
            CurrentRPM = InWheelRPM * Setup().OverallRatio;
        }
    }

    float GetWheelTorque()
    {
        return Throttle * GetTorqueFromRPM(CurrentRPM);
    }

    float GetTorqueFromRPM(float RPM);

    /* Get motor speed in Revolutions per Minute */
    float GetMotorRPM() const
    {
        return CurrentRPM;
    }

    TSet<int> GetWheelIndicies()
    {
        return Setup().WheelIndicies;
    }

    void Simulate(float DeltaTime);

protected:
    float Throttle; // [-1,1] normalized throttle
    float CurrentRPM;  // Wheel's Ground Speed
    float TargetSpeed; // target wheel speed [RPM]
    float DriveTorque;  // current torque [N.m]

    float IntegralAcc; // Accumulated Integral Term
    float PrevError;   // prev error for derivative term

    bool FreeRunning;   // is motor in neutral with no load

    float Omega;        // motor speed

    
};
