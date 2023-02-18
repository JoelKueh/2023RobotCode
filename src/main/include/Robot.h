// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <frc/Compressor.h>

#include "Drive.h"
#include "Arm.h"
#include "WiringDiagram.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  
  Drive *m_Drive;
  Arm *m_Arm;

  frc::XboxController Xbox { WiringDiagram::xboxPort };
  double xboxLX = 0;
  double xboxLY = 0;
  double xboxRX = 0;
  bool xboxRightBumper = false;

  frc::GenericHID ButtonBoard { WiringDiagram::buttonBoardPort };
  bool button1 = false;
  bool button2 = false;
  bool button3 = false;
  bool button4 = false;
  bool button5 = false;
  bool button6 = false;

  frc::Compressor Compressor { WiringDiagram::pneumaticsHubID, frc::PneumaticsModuleType::REVPH };

  void GetXbox();
  void GetButtonBoard();
};
