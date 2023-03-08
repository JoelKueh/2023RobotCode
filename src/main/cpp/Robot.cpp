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

void Robot::RobotPeriodic() {}

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

  if(xboxRightTrigger)
  {
    m_Drive->MecanumDrive(xboxLY/4, -xboxLX/4, -xboxRX/4);
  }
  else
  {
    m_Drive->MecanumDrive(xboxLY, -xboxLX, -xboxRX);
  }
  
  if(xboxRightBumper)
  {
    m_Arm->Toggle();
  }

  frc::SmartDashboard::PutBoolean("B1", button1);
  frc::SmartDashboard::PutBoolean("B2", button2);
  frc::SmartDashboard::PutBoolean("B3", button3);
  frc::SmartDashboard::PutBoolean("B4", button4);
  frc::SmartDashboard::PutBoolean("Manual Control", manualcontrol);

  if(button5 && button6)
  {
    manualcontrol = !manualcontrol;
  }

  if(manualcontrol)
  {
    m_Arm->ArmManual(joyY);
  }
  else
  {
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
    m_Arm->ArmUpdatePID();
  }
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
  if(Xbox.GetRightTriggerAxis() > .5)
  {
    xboxRightTrigger = true;
  }
  else
  {
    xboxRightTrigger = false;
  }
}

void Robot::GetButtonBoard()
{
  button1 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button1ID);
  button2 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button2ID);
  button3 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button3ID);
  button4 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button4ID);
  button5 = ButtonBoard.GetRawButton(WiringDiagram::button5ID);
  button6 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button6ID);
  joyY = ButtonBoard.GetRawAxis(WiringDiagram::joyYID);
}


#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
