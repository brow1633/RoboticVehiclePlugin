#include "MotorSystem.h"

FSimpleMotorSim::FSimpleMotorSim(const FSimpleMotorConfig *StaticDataIn) :
    TVehicleSystem<FSimpleMotorConfig>(StaticDataIn),
    GroundSpeed(0.f),
    CurrentRPM(0.f),
    TargetSpeed(0.f),
    DriveTorque(0.f),
    IntegralAcc(0.f),
    PrevError(0.f),
    FreeRunning(false),
    Omega(0.f)
{
}

float FSimpleMotorSim::GetTorqueFromVoltage(float MotorVoltageIn, float RPM)
{
    if(FreeRunning)
    {
        return -Setup().MotorSystemFriction * FMath::Sign(RPM);
    }

    float backEMF = (Setup().EMFConstant * Chaos::RPMToOmega(RPM));

    UE_LOG(LogTemp, Warning, TEXT("MotorVoltageIn: %lf, BackEMF: %lf"), MotorVoltageIn, backEMF);
    return (MotorVoltageIn - backEMF) / (Setup().ArmatureResistance) * Setup().TorqueConstant - Setup().MotorSystemFriction*FMath::Sign(RPM);
}

void FSimpleMotorSim::Simulate(float DeltaTime)
{

    //Omega += (TargetSpeed - Omega) * 4.0f * DeltaTime; // Fudge factor?
    Omega = Chaos::RPMToOmega(GroundSpeed);
    CurrentRPM = GroundSpeed;

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
    UE_LOG(LogTemp, Warning, TEXT("Target Speed: %lf, Setting Throttle: %lf"), TargetSpeed, throttle_signal);

    PrevError = error;

}
