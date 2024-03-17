// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AprilTagMapper2024.h"
#include <iostream>

//Errors
AprilTagMapper2024::MapResult Error_NoTargets = {false, "No targets found!"};
AprilTagMapper2024::MapResult Error_WrongPipeline = {false, "PhotonVision returned a target that was not an April Tag! Check pipeline mode"};
AprilTagMapper2024::MapResult Error_WrongTargets = {false, "Not seeing the correct targets for the current state!"};

AprilTagMapper2024::AprilTagMapper2024(frc::Transform3d cameraToRobot) {
    this->cameraToRobot = cameraToRobot;
    theoreticalTagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
};

void AprilTagMapper2024::SetCamera(photon::PhotonCamera* camera) {
    this->camera = camera;
}

AprilTagMapper2024::MapResult AprilTagMapper2024::CalculateEmpiricalLocations() {
    photon::PhotonPipelineResult pipelineRes = camera->GetLatestResult();
        
    if(!pipelineRes.HasTargets()) {
        return Error_NoTargets;
    }

    auto targets = pipelineRes.GetTargets();

    for(auto target : targets) {
        int id = target.GetFiducialId();

        if(id == -1) {
            return Error_WrongPipeline;
        }

        empiricalLocations[id] = target.GetBestCameraToTarget() + cameraToRobot.Inverse();
    }

    return {true, ""};
}

void AprilTagMapper2024::GenerateFieldJsonFromEmpiricalLocations(std::string jsonTargetPath) {
    auto theoreticalAprilTags = theoreticalTagLayout.GetTags();

    // Order tags by id, 1 is index 0
    std::sort(theoreticalAprilTags.begin(), theoreticalAprilTags.end(), [](auto a, auto b) { return a.ID < b.ID;});

    for(auto empiricalLocation : empiricalLocations) {

        if(!theoreticalLocations.contains(empiricalLocation.first)){
            std::cout << "JSON generation failed! Dont have a transform for ID: " << empiricalLocation.first << std::endl;
        }

        auto transFormDelta = theoreticalLocations[empiricalLocation.first] + empiricalLocation.second.Inverse();
        std::cout << "April Tag ID: " << empiricalLocation.first << " shifted by (x: " << transFormDelta.X().value() << ", y: " << transFormDelta.Y().value() << ", z: " << transFormDelta.Z().value();
        std::cout << ") (r: " << transFormDelta.Rotation().X().value() << ", p: " << transFormDelta.Rotation().Y().value() << ", y: " << transFormDelta.Rotation().Z().value() << ")" << std::endl;
        
        theoreticalAprilTags[empiricalLocation.first - 1].pose = theoreticalAprilTags[empiricalLocation.first - 1].pose.TransformBy(transFormDelta);
    }

    frc::AprilTagFieldLayout newLayout {theoreticalAprilTags, theoreticalTagLayout.GetFieldLength(), theoreticalTagLayout.GetFieldWidth()};

    newLayout.Serialize(jsonTargetPath);
}