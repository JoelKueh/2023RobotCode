// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/ArmFeedforward.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <units/time.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include "WiringDiagram.h"

#define ROT_TO_RAD(X) X * 3.141593 * 2 / 88
#define RAD_TO_ROT(X) X * 88 / 3.141593 / 2
#define M_PI 3.141593

class Arm {
 public:
  Arm();

  void Toggle();
  void Closed();
  void Open();
  void SetSetpoint(double position);
  bool ZeroArm();
  void ArmManual(double speed);
  void ArmUpdatePID();

  // The setpoint measured from the vertical.
  units::radian_t angle_offset {M_PI / 2};
  units::radian_t setpoint {0.0};
  
  units::volt_t kS {0.0};
  units::volt_t kG {0.87};

  rev::CANSparkMax armMotor{WiringDiagram::armMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController armPID = armMotor.GetPIDController();
  rev::SparkMaxRelativeEncoder armEncoder = armMotor.GetEncoder();
  frc::DigitalInput armLimit{WiringDiagram::armLimitID};
  frc::ArmFeedforward armFF{kS, kG, 1.71_V * 1_s / 1_rad, 0.06_V * 1_s * 1_s / 1_rad};
  frc::TrapezoidProfile<units::radians> *TP;
  frc::Timer timer;

  double kP = 0.0, kI = 0.0, kD = 0.0, kIz = 0.0, kFF = 0.0, kMaxOutput = 0.1, kMinOutput = -0.1;
  double inrobot = ROT_TO_RAD(0.5),
    goal2 = ROT_TO_RAD(18.316),
    substation = ROT_TO_RAD(19.037),
    goal3 = ROT_TO_RAD(21.72);

 private:
  frc::DoubleSolenoid ClawPiston { WiringDiagram::pneumaticsHubID, frc::PneumaticsModuleType::REVPH, WiringDiagram::solenoidForwardID, WiringDiagram::solenoidReverseID};
};
