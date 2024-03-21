// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>

#include "AprilTagMapper2024/AprilTagMapper2024.h"

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
  frc2::CommandPtr MapField();

 private:
  AprilTagMapper2024 mapper {{ {0_m, 0_m, 0.355_m}, {0_deg, -26_deg, 0_deg} }};
  photon::PhotonCamera camera {"Arducam_OV9281_USB_Camera"};

  frc2::CommandPtr mapFieldCmd = MapField();
};
