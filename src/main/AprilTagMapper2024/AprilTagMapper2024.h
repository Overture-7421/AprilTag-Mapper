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

  MapResult CalculateEmpiricalLocations();

  void GenerateFieldJsonFromEmpiricalLocations(std::string jsonTargetPath);

private:
  photon::PhotonCamera* camera;
	frc::Transform3d cameraToRobot;

	frc::AprilTagFieldLayout theoreticalTagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo) };

  std::map<int, frc::Transform3d> theoreticalLocations {
    // Blue
    {7, AprilTagMapper2024Constants::BlueSpeakerWaypointPose - theoreticalTagLayout.GetTagPose(7).value()},
    {8, AprilTagMapper2024Constants::BlueSpeakerWaypointPose - theoreticalTagLayout.GetTagPose(8).value()},
    {6, AprilTagMapper2024Constants::BlueAmpWaypointPose - theoreticalTagLayout.GetTagPose(6).value()},
    {14, AprilTagMapper2024Constants::BlueStageBackWaypointPose - theoreticalTagLayout.GetTagPose(14).value()},

    // Red
    {4, AprilTagMapper2024Constants::RedSpeakerWaypointPose - theoreticalTagLayout.GetTagPose(4).value()},
    {3, AprilTagMapper2024Constants::RedSpeakerWaypointPose - theoreticalTagLayout.GetTagPose(3).value()},
    {5, AprilTagMapper2024Constants::RedAmpWaypointPose - theoreticalTagLayout.GetTagPose(5).value()},
    {13, AprilTagMapper2024Constants::RedStageBackWaypointPose - theoreticalTagLayout.GetTagPose(13).value()}
  };


  std::map <int, frc::Transform3d> empiricalLocations;
};
