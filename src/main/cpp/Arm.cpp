// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Arm.h"

Arm::Arm()
{
    armMotor.SetInverted(false);
    armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    armMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, RAD_TO_ROT(2));
    armMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    armMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0.05);

    timer.Start();

    armPID.SetP(kP);
    armPID.SetI(kI);
    armPID.SetD(kD);
    armPID.SetIZone(kIz);
    armPID.SetFF(kFF);
    armPID.SetOutputRange(kMinOutput, kMaxOutput);

    TP = new frc::TrapezoidProfile<units::radians>(
        frc::TrapezoidProfile<units::radians>::Constraints{1_rad_per_s, 1_rad_per_s / 1_s},
        frc::TrapezoidProfile<units::radians>::State{(units::radian_t)inrobot, 0.0_rad_per_s},
        frc::TrapezoidProfile<units::radians>::State{0.0_rad, 0.0_rad_per_s}
    );
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

void Arm::SetSetpoint(int position)
{
    // If you don't haave this, you never get rid of the old profile.
    // This is called a memory leak, it slowly eats away at your memory until
    // the program crashes by taking away chunks of ram at a time each time
    // this function is run.
    delete TP;

    if(position == 1)
    {
        setpoint = units::radian_t(inrobot);
    }
    else if(position == 2)
    {
        setpoint = units::radian_t(goal2);
    }
    else if(position == 3)
    {
        setpoint = units::radian_t(substation);
    }
    else if(position == 4)
    {
        setpoint = units::radian_t(goal3);
    }
    else if(position == 5)
    {
        setpoint = units::radian_t(rampPosition);
    }

    frc::SmartDashboard::PutNumber("Setpoint", setpoint.value());
    // Set the time to zero.
    timer.Reset();
    // Make the new profile.
    TP = new frc::TrapezoidProfile<units::radians>(
        frc::TrapezoidProfile<units::radians>::Constraints{1_rad_per_s, 1_rad_per_s / 1_s},
        frc::TrapezoidProfile<units::radians>::State{(units::radian_t)setpoint, 0.0_rad_per_s},
        frc::TrapezoidProfile<units::radians>::State{(units::radian_t)(ROT_TO_RAD(armEncoder.GetPosition())), 0.0_rad_per_s}
    );
}

void Arm::ArmUpdatePID()
{
    frc::SmartDashboard::PutNumber("Time Since TP Creation", timer.Get().value());
    auto processVar = TP->Calculate(timer.Get());
    auto feedForward = armFF.Calculate(processVar.position - angle_offset, processVar.velocity);

    frc::SmartDashboard::PutNumber("Process Variable", processVar.position());
    frc::SmartDashboard::PutNumber("Position", ROT_TO_RAD(armEncoder.GetPosition()));
    frc::SmartDashboard::PutNumber("Voltage Out", feedForward.value());
    frc::SmartDashboard::PutNumber("Error", ROT_TO_RAD(armEncoder.GetPosition())
        - processVar.position());

    armPID.SetReference(RAD_TO_ROT(processVar.position.value()), rev::CANSparkMax::ControlType::kPosition,
        0, feedForward.value());
}

bool Arm::ZeroArm()
{
    armMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    if(armLimit.Get())
    {
        armMotor.Set(-0.04);
        return false;
    }
    else
    {
        armMotor.Set(0);
        armEncoder.SetPosition(0);
        armMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        return true;
    }
}

void Arm::ArmManual(double speed)
{
    armMotor.Set(speed/10);
}