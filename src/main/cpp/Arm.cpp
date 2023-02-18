// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Arm.h"

Arm::Arm()
{
    armMotor.SetInverted(true);

    armPID.SetP(kP);
    armPID.SetI(kI);
    armPID.SetD(kD);
    armPID.SetIZone(kIz);
    armPID.SetFF(kFF);
    armPID.SetOutputRange(kMinOutput, kMaxOutput);
}
    
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

void Arm::ArmPosition(double position)
{
    if(position == 1)
    {
        armPID.SetReference(pos1, rev::CANSparkMax::ControlType::kPosition);
    }
    else if(position == 2)
    {
        armPID.SetReference(pos2, rev::CANSparkMax::ControlType::kPosition);
    }
    else if(position == 3)
    {
        armPID.SetReference(pos3, rev::CANSparkMax::ControlType::kPosition);
    }
    else if(position == 4)
    {
        armPID.SetReference(pos4, rev::CANSparkMax::ControlType::kPosition);
    }
}

bool Arm::ZeroArm()
{
    if(armLimit.Get())
    {
        armMotor.Set(-0.1);
        return false;
    }
    else
    {
        armMotor.Set(0);
        return true;
    }
}

void Arm::ArmManual(double speed)
{
    armMotor.Set(speed/4);
}