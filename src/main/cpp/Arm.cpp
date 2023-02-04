// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Arm.h"

Arm::Arm() = default;
    
    void Arm::Toggle()
    {
        ClawPiston.Toggle();
    }

    void Arm::Closed()
    {
        ClawPiston.Set(frc::DoubleSolenoid::kForward);
    }

    void Arm::Open()
    {
        ClawPiston.Set(frc::DoubleSolenoid::kReverse);
    }