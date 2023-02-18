// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include "WiringDiagram.h"

class Arm {
 public:
  Arm();

  void Toggle();
  void Closed();
  void Open();
  void ArmPosition(double position);
  bool ZeroArm();

  rev::CANSparkMax armMotor{WiringDiagram::armMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController armPID = armMotor.GetPIDController();
  rev::SparkMaxRelativeEncoder armEncoder = armMotor.GetEncoder();
  frc::DigitalInput armLimit{WiringDiagram::armLimitID};

  double kP = 0.0, kI = 0.0, kD = 0.0, kIz = 0.0, kFF = 0.0, kMaxOutput = 0.0, kMinOutput = 0.0;
  double pos1 = 0.0, pos2 = 0.0, pos3 = 0.0, pos4 = 0.0;

 private:
  frc::DoubleSolenoid ClawPiston {frc::PneumaticsModuleType::REVPH, WiringDiagram::solenoidForwardID, WiringDiagram::solenoidReverseID};
};
