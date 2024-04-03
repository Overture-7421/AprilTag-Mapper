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
  frc2::CommandPtr GenerateFieldJson();
  frc2::CommandPtr TakeMapperSnapshot();

  frc2::CommandPtr MapFieldBlueSpeaker();
  frc2::CommandPtr MapFieldBlueStage();
  frc2::CommandPtr MapFieldRedSpeaker();
  frc2::CommandPtr MapFieldRedStage();

  frc2::CommandPtr LoadCustomLayout();

 private:
  AprilTagMapper2024 mapper {{ {0_m, 0_m, 0.3518230454_m}, {180_deg, -23_deg, 0_deg} }};
  photon::PhotonCamera camera {"Arducam_OV2311_USB_Camera"};

  frc2::CommandPtr mapBlueSpeakerCmd = MapFieldBlueSpeaker().AndThen(GenerateFieldJson()).IgnoringDisable(true);
  frc2::CommandPtr mapBlueStageCmd = MapFieldBlueStage().AndThen(GenerateFieldJson()).IgnoringDisable(true);
  frc2::CommandPtr mapRedSpeakerCmd = MapFieldRedSpeaker().AndThen(GenerateFieldJson()).IgnoringDisable(true);
  frc2::CommandPtr mapRedStageCmd = MapFieldRedStage().AndThen(GenerateFieldJson()).IgnoringDisable(true);
  frc2::CommandPtr mapFullFieldCmd = MapFieldBlueSpeaker().AndThen(MapFieldBlueStage()).AndThen(MapFieldRedStage()).AndThen(MapFieldRedSpeaker()).AndThen(GenerateFieldJson()).IgnoringDisable(true);

  frc2::CommandPtr loadCustomTagLayoutCmd = LoadCustomLayout().IgnoringDisable(true);
};
