// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RomiDrivetrain.h"
#include "Constants.h"
#include "units/angle.h"
#include "units/base.h"
#include "units/length.h"
#include "units/time.h"
#include "frc/geometry/Rotation2d.h"

RomiDrivetrain::RomiDrivetrain() {
  leftEncoder.SetDistancePerPulse(DrivetrainConstants::kDistPerPulseDouble);
  rightEncoder.SetDistancePerPulse(DrivetrainConstants::kDistPerPulseDouble);

  rightSpark.SetInverted(true);
}

units::radian_t RomiDrivetrain::GetAngle() {
  return gyro.GetAngle();
}

frc::Rotation2d RomiDrivetrain::GetRot2d() {
  return frc::Rotation2d{GetAngle()};
}

units::meter_t RomiDrivetrain::GetLeftDist() {
  return units::length::meter_t(leftEncoder.GetDistance());
}

units::meter_t RomiDrivetrain::GetRightDist() {
  return units::length::meter_t(rightEncoder.GetDistance());
}

units::meter_t RomiDrivetrain::GetAveDist() {
  return (GetRightDist() + GetLeftDist()) / 2;
}

units::meters_per_second_t RomiDrivetrain::GetLeftSpeed() {
  return units::meters_per_second_t(leftEncoder.GetRate());
}

units::meters_per_second_t RomiDrivetrain::GetRightSpeed() {
  return units::meters_per_second_t(rightEncoder.GetRate());
}

units::meters_per_second_t RomiDrivetrain::GetAveSpeed() {
  return (GetRightSpeed() + GetLeftSpeed()) / 2;
}

void RomiDrivetrain::ArcadeDrive(double xaxisSpeed, double zaxisRotate) {
  diffDrive.ArcadeDrive(xaxisSpeed, zaxisRotate, true);
}

void RomiDrivetrain::ResetEncodersAndGyro() {
  leftEncoder.Reset();
  rightEncoder.Reset();
  gyro.Reset();
}
