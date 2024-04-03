// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

std::string GetJsonPath() {
  #ifndef __FRC_ROBORIO__
  return "7421-field.json";
  #else
  return "/home/lvuser/7421-field.json";
  #endif
}

frc2::CommandPtr ContinueNT() {
  return frc2::cmd::Sequence(
    frc2::cmd::WaitUntil([] {
      return frc::SmartDashboard::GetBoolean("Continue", false);
    }),
    frc2::cmd::Wait(0.15_s),
    frc2::cmd::RunOnce([] {
      frc::SmartDashboard::PutBoolean("Continue", false);
    })
  );
}

void Robot::RobotInit() {
  mapper.SetCamera(&camera);
  frc::SmartDashboard::PutData("Map Blue Speaker", mapBlueSpeakerCmd.get());
  frc::SmartDashboard::PutData("Map Blue Stage", mapBlueStageCmd.get());
  frc::SmartDashboard::PutData("Map Red Speaker", mapRedSpeakerCmd.get());
  frc::SmartDashboard::PutData("Map Red Stage", mapRedStageCmd.get());
  frc::SmartDashboard::PutData("Map Full Field", mapFullFieldCmd.get());

  frc::SmartDashboard::PutData("Load Custom Layout", loadCustomTagLayoutCmd.get());

  frc::SmartDashboard::PutBoolean("Continue", false); 
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

frc2::CommandPtr Robot::TakeMapperSnapshot() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Taking snapshot..."),
    frc2::cmd::RunOnce([&] {
        mapper.CalculateEmpiricalLocations();
    })
  );
}

frc2::CommandPtr Robot::GenerateFieldJson() {
  return frc2::cmd::RunOnce([&] {
    mapper.GenerateFieldJsonFromEmpiricalLocations(GetJsonPath());
  });
}

frc2::CommandPtr Robot::MapFieldBlueSpeaker() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Aim the camera at the blue speaker..."),
    ContinueNT(),
    TakeMapperSnapshot(),
    frc2::cmd::Print("Aim the camera at the blue amp..."),
    ContinueNT(),
    TakeMapperSnapshot()
  );
}

frc2::CommandPtr Robot::MapFieldBlueStage() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Aim the camera at the blue stage..."),
    ContinueNT(),
    TakeMapperSnapshot()
  );
}

frc2::CommandPtr Robot::MapFieldRedSpeaker() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Aim the camera at the red speaker..."),
    ContinueNT(),
    TakeMapperSnapshot(),
    frc2::cmd::Print("Aim the camera at the red amp..."),
    ContinueNT(),
    TakeMapperSnapshot()
  );
}

frc2::CommandPtr Robot::MapFieldRedStage() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Aim the camera at the red stage..."),
    ContinueNT(),
    TakeMapperSnapshot()
  );
}

frc2::CommandPtr Robot::LoadCustomLayout() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Loading custom tag layout from ./7421-field.json"),
    frc2::cmd::RunOnce([&] {
      mapper.SetAprilTagLayout(frc::AprilTagFieldLayout("./7421-field.json"));
    })
  );
}

#ifndef RUNNING_FRC_TESTS
int main() {
  photon::PhotonCamera::SetVersionCheckEnabled(false);
  return frc::StartRobot<Robot>();
}
#endif
