// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Drive/MecanumDrive.h>
#include <rev/CANSparkMax.h>
#include "WiringDiagram.h"

class Drive {
 public:
  Drive();
  void MecanumDrive(double drivePower, double strafePower, double turnPower);

  rev::CANSparkMax frontLeftMotor { WiringDiagram::frontLeftID, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax frontRightMotor { WiringDiagram::frontRightID, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax backLeftMotor { WiringDiagram::backLeftID, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax backRightMotor { WiringDiagram::backRightID, rev::CANSparkMax::MotorType::kBrushless };
  frc::MecanumDrive *myMecanumDrive;
};
