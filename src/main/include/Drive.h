// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Drive/MecanumDrive.h>
#include <rev/CANSparkMax.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/length.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "WiringDiagram.h"

class Drive {
 public:
  Drive();

  void MecanumDrive(double drivePower, double strafePower, double turnPower);
  double MDPID();
  double MDPID2();
  void autoReset();
  void autoReset2();
  void autoReset3();
  void autoReset4();
  void autoReset5();
  void autoReset6();
  void autoReset7();
  void checkAutoState();

  double kP = 0.3, kI = 0.0, kD = 0.0;
  double kP2 = 0.8, kI2 = 0.04, kD2 = 0.0;
  int autostate = 0;
  bool runonce = true;
  bool arm4 = false;
  bool arm1 = false;
  bool armrunonce = true;

  rev::CANSparkMax frontLeftMotor {WiringDiagram::frontLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor {WiringDiagram::frontRightID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backLeftMotor {WiringDiagram::backLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backRightMotor {WiringDiagram::backRightID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder frontRightEncoder = frontRightMotor.GetEncoder();
  frc::MecanumDrive *myMecanumDrive;
  frc::ADIS16470_IMU gyro;
  frc::TrapezoidProfile<units::feet> *autoTP;
  frc::TrapezoidProfile<units::radians> *gyroTP;
  frc::Timer autotimer;
  frc2::PIDController mecanumPID{kP, kI, kD};
  frc2::PIDController gyroPID{kP2, kI2, kD2};
  frc::DigitalInput autoswitch{WiringDiagram::autoswitchID};

private:
  inline double feet_to_rot(double feet) {
    return feet * 6.8;
  }
  inline double rot_to_feet(double rot) {
    return rot * 0.14706;
  }
  inline double deg_to_rad(double deg) {
    return deg * 0.0174533;
  }
  inline double rad_to_deg(double rad) {
    return rad * 57.2958;
  }
};
