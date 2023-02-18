// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"

Drive::Drive()
{
    frontLeftMotor.SetInverted(true);
    backLeftMotor.SetInverted(true);

    myMecanumDrive = new frc::MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
}

void Drive::MecanumDrive(double drivePower, double strafePower, double turnPower)
{
    double driveOut = drivePower*drivePower;
    driveOut = driveOut < 0.2 ? 0.2 : driveOut;
    driveOut = drivePower < 0.0 ? -driveOut : driveOut;

    double strafeOut = strafePower*strafePower;
    strafeOut = strafeOut < 0.2 ? 0.2 : strafeOut;
    strafeOut = strafePower < 0.0 ? -strafeOut : strafeOut;

    double turnOut = turnPower*turnPower;
    turnOut = turnOut < 0.2 ? 0.2 : turnOut;
    turnOut = turnPower < 0.0 ? -turnOut : turnOut;

    driveOut = drivePower < 0.2 && drivePower > -0.2 ? 0.0 : driveOut;
    strafeOut = strafePower < 0.2 && strafePower > -0.2 ? 0.0 : strafeOut;
    turnOut = turnPower < 0.2 && turnPower > -0.2 ? 0.0 : turnOut;

    myMecanumDrive->DriveCartesian(driveOut, strafeOut, turnOut);
}