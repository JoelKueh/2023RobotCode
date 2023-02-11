// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include "WiringDiagram.h"

class Arm {
 public:
  Arm();

  void Toggle();
  void Closed();
  void Open();

  rev::CANSparkMax armMotor{ WiringDiagram::armMotorID, rev::CANSparkMax::MotorType::kBrushless};
  void SetPosition(double position);

 private:
  frc::DoubleSolenoid ClawPiston {frc::PneumaticsModuleType::REVPH, WiringDiagram::solenoidForwardID, WiringDiagram::solenoidReverseID};
};
