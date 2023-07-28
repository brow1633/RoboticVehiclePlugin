#include "MotorSystem.h"

FSimpleMotorSim::FSimpleMotorSim(const FSimpleMotorConfig *StaticDataIn) :
    TVehicleSystem<FSimpleMotorConfig>(StaticDataIn),
    CurrentRPM(0.f),
    TargetSpeed(0.f),
    DriveTorque(0.f),
    IntegralAcc(0.f),
    PrevError(0.f),
    FreeRunning(false),
    Omega(0.f)
{
}

float FSimpleMotorSim::GetTorqueFromRPM(float RPM)
{
    if(FMath::Abs(RPM) > 10 * Setup().MaxRpm) {
        UE_LOG(LogTemp, Warning, TEXT("Max RPM greatly exceeded"));
        return 0.0;
    }

    if(FreeRunning)
    {
        return -Setup().MotorSystemFriction * FMath::Sign(RPM);
    }


    UE_LOG(LogTemp, Warning, TEXT("GetTorqueFromRPM: %lf"), RPM);
    float Torque = Setup().TorqueCurve.GetValue(FMath::Abs(RPM), Setup().MaxRpm, Setup().MaxTorque);

    //UE_LOG(LogTemp, Warning, TEXT("MotorVoltageIn: %lf, BackEMF: %lf"), MotorVoltageIn, backEMF);
    return Setup().OverallRatio * Torque;
}

void FSimpleMotorSim::Simulate(float DeltaTime)
{
    if(Setup().PidControl)
    {
        //Omega += (TargetSpeed - Omega) * 4.0f * DeltaTime; // Fudge factor?
        Omega = Chaos::RPMToOmega(CurrentRPM);

        float error = TargetSpeed - CurrentRPM;

        // Reset Integral
        if(signbit(IntegralAcc) == signbit(error))
        {
            IntegralAcc += error * DeltaTime;
        }
        else
        {
            IntegralAcc = 0;
        }

        float derivative = (error - PrevError) / DeltaTime;

        float throttle_signal = 
                Setup().ProportionalGain * error +
                Setup().IntegralGain * IntegralAcc +
                Setup().DerivativeGain * derivative;

        SetThrottle(throttle_signal);
        UE_LOG(LogTemp, Warning, TEXT("Target Speed: %lf, Error: %lf, Setting Throttle: %lf"), TargetSpeed, error, throttle_signal);

        PrevError = error;
    }

}
