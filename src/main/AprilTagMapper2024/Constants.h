#pragma once
#include <frc/geometry/Pose3d.h>

namespace AprilTagMapper2024Constants {
    // Blue
    static const frc::Pose3d BlueSpeakerWaypointPose {2.77_m, 5.54_m, 0_m, {0_deg, 0_deg, 180_deg}};
    static const frc::Pose3d BlueAmpWaypointPose {2.77_m, 5.54_m, 0_m, {0_deg, 0_deg, 90_deg}};
    static const frc::Pose3d BlueStageBackWaypointPose {8.15_m, 4.10_m, 0_m, {0_deg, 0_deg, 180_deg}};

    // Red
    static const frc::Pose3d RedSpeakerWaypointPose {13.81_m, 5.54_m, 0_m, {0_deg, 0_deg, 0_deg}};
    static const frc::Pose3d RedAmpWaypointPose {13.81_m, 5.54_m, 0_m, {0_deg, 0_deg, 90_deg}};
    static const frc::Pose3d RedStageBackWaypointPose {8.4_m, 4.10_m, 0_m, {0_deg, 0_deg, 0_deg}};
};