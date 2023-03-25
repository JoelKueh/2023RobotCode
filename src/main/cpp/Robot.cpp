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
  m_Arm->SetSetpoint(1);
  m_Drive->autoReset();
  m_Drive->autostate = 0;
  m_Drive->armrunonce = true;
  m_Drive->runonce = true;
  autoresetdone = false;
}

void Robot::AutonomousPeriodic() 
{
  if(!autoresetdone)
  {
    autoresetdone = m_Arm->ZeroArm();
    m_Drive->MecanumDrive(0, 0, 0);
    return;
  }
  NonRampAuto();
}

void Robot::TeleopInit() 
{
  m_Arm->Open();
  m_Arm->SetSetpoint(1);
  resetdone = true;
}

void Robot::TeleopPeriodic()
{
  GetButtonBoard();
  GetXbox();

  if(button6)
  {
    resetdone = !resetdone;
  }
  if(!resetdone)
  {
    resetdone = m_Arm->ZeroArm();
    m_Drive->MecanumDrive(0.0, 0.0, 0.0);
    return;
  }

  if(xboxRightTrigger)
  {
    m_Drive->MecanumDrive(xboxLY/4, -xboxLX/2, -xboxRX/4);
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

  if(selectbutton && startbutton)
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
    else if(button5)
    {
      m_Arm->SetSetpoint(5);
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
  button5 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button5ID);
  button6 = ButtonBoard.GetRawButtonPressed(WiringDiagram::button6ID);
  selectbutton = ButtonBoard.GetRawButton(WiringDiagram::selectbuttonID);
  startbutton = ButtonBoard.GetRawButtonPressed(WiringDiagram::startbuttonID);
  joyY = ButtonBoard.GetRawAxis(WiringDiagram::joyYID);
}

void Robot::NonRampAuto()
{
  if(m_Drive->autostate == 0)
  {
    m_Drive->MecanumDrive(m_Drive->MDPID(), 0, 0);
    if(m_Drive->arm4)
    {
      m_Arm->SetSetpoint(4);
      m_Drive->arm4 = false;
    }
  }
  else if(m_Drive->autostate == 1)
  {
    if(m_Drive->runonce)
    {
      m_Drive->autoReset2();
      m_Drive->runonce = false;
    }
    m_Drive->MecanumDrive(m_Drive->MDPID(), 0, 0);
  }
  else if(m_Drive->autostate == 2)
  {
    if(m_Drive->runonce)
    {
      m_Arm->Open();
      m_Drive->autoReset3();
      m_Drive->runonce = false;
    }
    m_Drive->MecanumDrive(m_Drive->MDPID(), 0, 0);
    if(m_Drive->arm1)
    {
      m_Arm->SetSetpoint(5);
      m_Drive->arm1 = false;
    }
  }
  // else if(m_Drive->autostate == 3)
  // {
  //   if(m_Drive->runonce)
  //   {
  //     m_Drive->autoReset4();
  //     m_Drive->runonce = false;
  //   }
  //   m_Drive->MecanumDrive(0, 0, m_Drive->MDPID2());
  // }
  // else if(m_Drive->autostate == 4)
  // {
  //   if(m_Drive->runonce)
  //   {
  //     m_Drive->autoReset5();
  //     m_Drive->runonce = false;
  //   }
  //   m_Drive->MecanumDrive(m_Drive->MDPID(), 0, 0);
  // }
  // else if(m_Drive->autostate == 5)
  // {
  //   if(m_Drive->runonce)
  //   {
  //     m_Arm->Closed();
  //     m_Drive->autoReset6();
  //     m_Drive->runonce = false;
  //   }
  //   m_Drive->MecanumDrive(0, 0, m_Drive->MDPID2());
  // }
  // else if(m_Drive->autostate == 6)
  // {
  //   if(m_Drive->runonce)
  //   {
  //     m_Arm->SetSetpoint(4);
  //     m_Drive->autoReset7();
  //     m_Drive->runonce = false;
  //   }
  //   m_Drive->MecanumDrive(m_Drive->MDPID(), 0, 0);
  // }
  m_Arm->ArmUpdatePID();
  m_Drive->checkAutoState();
  frc::SmartDashboard::PutNumber("autostate", m_Drive->autostate);
  frc::SmartDashboard::PutBoolean("runonce", m_Drive->runonce);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
