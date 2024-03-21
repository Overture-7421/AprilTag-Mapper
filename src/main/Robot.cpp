// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  photon::PhotonCamera::SetVersionCheckEnabled(false);
  mapper.SetCamera(&camera);
  frc::SmartDashboard::PutData("Map Field", mapFieldCmd.get());
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


frc2::CommandPtr ContinueNT() {

  return frc2::cmd::Sequence(
    frc2::cmd::WaitUntil([] {
      return frc::SmartDashboard::GetBoolean("Continue", false);
    }),
    frc2::cmd::Wait(0.5_s)
  );
}

frc2::CommandPtr TakeMapperSnapshot(AprilTagMapper2024* mapper) {
  return frc2::cmd::RunOnce([=] {
    mapper->CalculateEmpiricalLocations();
  });
}

frc2::CommandPtr GenerateFieldJson(AprilTagMapper2024* mapper) {
  return frc2::cmd::RunOnce([=] {
    mapper->GenerateFieldJsonFromEmpiricalLocations("/home/lvuser/7421-field.json");
  });
}

frc2::CommandPtr Robot::MapField() {
  return frc2::cmd::Sequence(
    frc2::cmd::Print("Aim the camera at the speaker..."),
    ContinueNT(),
    frc2::cmd::Print("Taking snapshot..."),
    TakeMapperSnapshot(&mapper),
    frc2::cmd::Print("Generating field..."),
    GenerateFieldJson(&mapper)
  );
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
