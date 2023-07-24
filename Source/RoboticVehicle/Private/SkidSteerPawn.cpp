// Fill out your copyright notice in the Description page of Project Settings.


#include "SkidSteerPawn.h"
#include "SkidSteerComponent.h"

ASkidSteerPawn::ASkidSteerPawn(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer.SetDefaultSubobjectClass<USkidSteerComponent>(VehicleMovementComponentName))
{}