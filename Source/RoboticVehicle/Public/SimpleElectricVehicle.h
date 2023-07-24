#include "SimpleVehicle.h"
#include "MotorSystem.h"

class FSimpleElectricWheeledVehicle : public Chaos::FSimpleWheeledVehicle
{
public:

    virtual ~FSimpleElectricWheeledVehicle() 
    {

    }

    bool HasMotor()
    {
        return (Motors.Num() > 0);
    }

    FSimpleMotorSim& GetMotor(int MotorIdx)
    {
        check(MotorIdx < Motors.Num());
        return Motors[MotorIdx];
    }

    TArray<FSimpleMotorSim> Motors;
};