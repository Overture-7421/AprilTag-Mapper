// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <map>
#include <frc/geometry/Translation3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>

#include "Constants.h"

class AprilTagMapper2024 {
 public:
  struct MapResult {
    bool succesful = false;
    std::string msg = "";
  };

  AprilTagMapper2024(frc::Transform3d cameraToRobot);
  
  void SetCamera(photon::PhotonCamera* camera);

  void SetAprilTagLayout(frc::AprilTagFieldLayout theoreticalTagLayout);

  MapResult CalculateEmpiricalLocations();

  void GenerateFieldJsonFromEmpiricalLocations(std::string jsonTargetPath);

private:
  photon::PhotonCamera* camera;
	frc::Transform3d cameraToRobot;

	frc::AprilTagFieldLayout theoreticalTagLayout;

  std::map<int, frc::Transform3d> theoreticalLocations;
  std::map <int, frc::Transform3d> empiricalLocations;
};
