// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/length.h"
#include "math.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {
inline constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants

namespace DrivetrainConstants {
inline constexpr double kCountsPerRevolution = 1440.0;
inline constexpr units::meter_t kWheelDiameter = 0.07_m;
inline constexpr units::meter_t kDistPerPulse = kWheelDiameter * (M_PI / kCountsPerRevolution);
inline constexpr double kDistPerPulseDouble = kDistPerPulse.value();

inline constexpr double kTranslationP = 0.1;
inline constexpr double kTranslationI = 0;
inline constexpr double kTranslationD = 0.05;

inline constexpr double kRotationP = 0.1;
inline constexpr double kRotationI = 0;
inline constexpr double kRotationD = 0.05;
}
