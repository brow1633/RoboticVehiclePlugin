// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "WheeledVehiclePawn.h"
#include "SkidSteerPawn.generated.h"



/**
 * 
 */
UCLASS(abstract, config=Game, BlueprintType)
class ROBOTICVEHICLE_API ASkidSteerPawn : public AWheeledVehiclePawn
{
public:
	GENERATED_BODY()
    ASkidSteerPawn(const FObjectInitializer &ObjectInitializer);
};
