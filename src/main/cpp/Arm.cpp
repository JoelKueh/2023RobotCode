// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Arm.h"

Arm::Arm()
{
    armMotor.SetInverted(true);
    armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armPID.SetP(kP);
    armPID.SetI(kI);
    armPID.SetD(kD);
    armPID.SetIZone(kIz);
    armPID.SetFF(kFF);
    armPID.SetOutputRange(kMinOutput, kMaxOutput);

    TP = new frc::TrapezoidProfile<units::radians>(
        frc::TrapezoidProfile<units::radians>::Constraints{1_rad_per_s, 2_rad_per_s / 1_s},
        frc::TrapezoidProfile<units::radians>::State{(units::radian_t)inrobot, 0.0_rad_per_s},
        frc::TrapezoidProfile<units::radians>::State{0.0_rad, 0.0_rad_per_s});
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
    delete TP;
    frc::
    if(position == 1)
    {
        armPID.SetReference(inrobot, rev::CANSparkMax::ControlType::kPosition);
    }
    else if(position == 2)
    {
        armPID.SetReference(goal2, rev::CANSparkMax::ControlType::kPosition);
    }
    else if(position == 3)
    {
        armPID.SetReference(substation, rev::CANSparkMax::ControlType::kPosition);
    }
    else if(position == 4)
    {
        armPID.SetReference(goal3, rev::CANSparkMax::ControlType::kPosition);
    }

    TP = new 
}

void Arm::ArmUpdatePID()
{
    double feedForward = armFF.Calculate(setpoint - angle_offset,
        )
}

void Arm::ArmSetSetpoint()
{

}

bool Arm::ZeroArm()
{
    if(armLimit.Get())
    {
        armMotor.Set(0.05);
        return false;
    }
    else
    {
        armMotor.Set(0);
        armEncoder.SetPosition(0);
        return true;
    }
}

void Arm::ArmManual(double speed)
{
    armMotor.Set(speed/4);
}