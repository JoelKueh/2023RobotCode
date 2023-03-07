// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  m_Drive = new Drive();
  m_Arm = new Arm();

  Compressor.EnableDigital();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_Arm->Closed();
}

void Robot::AutonomousPeriodic() 
{
  
}

void Robot::TeleopInit() 
{
  m_Arm->Closed();
  resetdone = false;
}

void Robot::TeleopPeriodic()
{
  if(!resetdone)
  {
    resetdone = m_Arm->ZeroArm();
    m_Drive->MecanumDrive(0, 0, 0);
    return;
  }

  GetXbox();
  GetButtonBoard();
  // m_Drive->MecanumDrive(xboxLY, -xboxLX, -xboxRX);
  m_Drive->MecanumDrive(0, 0, 0);

  if(xboxRightBumper)
  {
    // m_Arm->Toggle();
  }

  frc::SmartDashboard::PutBoolean("B1", button1);
  frc::SmartDashboard::PutBoolean("B2", button2);
  frc::SmartDashboard::PutBoolean("B3", button3);
  frc::SmartDashboard::PutBoolean("B4", button4);

  if(button1)
  {
    m_Arm->SetSetpoint(1);
  }
  else if(button2)
  {
    m_Arm->SetSetpoint(2);
  }
  else if(button3)
  {
    m_Arm->SetSetpoint(3);
  }
  else if(button4)
  {
    m_Arm->SetSetpoint(4);
  }
  frc::SmartDashboard::PutBoolean("Compressor", xboxRightBumper);
  m_Arm->ArmUpdatePID();
  
  // Arm can only be run in PID mode or Joystick mode at one time
  // This needs to be shoved in some switched if statment if manual control
  // is to be retained.
  // m_Arm->ArmManual(joyY);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void Robot::GetXbox()
{
  xboxLX = Xbox.GetLeftX();
  if(xboxLX < .2 && xboxLX > -.2)
  {
    xboxLX = 0;
  }

  xboxLY = Xbox.GetLeftY();
  if(xboxLY < .2 && xboxLY > -.2)
  {
    xboxLY = 0;
  }

  xboxRX = Xbox.GetRightX();
  if(xboxRX < .2 && xboxRX > -.2)
  {
    xboxRX = 0;
  }
  xboxRightBumper = Xbox.GetRightBumperPressed();
}

void Robot::GetButtonBoard()
{
  button1 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button1ID);
  button2 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button2ID);
  button3 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button3ID);
  button4 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button4ID);
  button5 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button5ID);
  button6 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button6ID);
  joyY = ButtonBoard.GetRawAxis(WiringDiagram::joyYID);
}


#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
