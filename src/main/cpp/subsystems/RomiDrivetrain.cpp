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

using namespace frc;
using namespace units;

RomiDrivetrain::RomiDrivetrain() {
  leftEncoder.SetDistancePerPulse(DrivetrainConstants::kDistPerPulseDouble);
  rightEncoder.SetDistancePerPulse(DrivetrainConstants::kDistPerPulseDouble);

  rightSpark.SetInverted(true);

  ResetAllComponents();
}

radian_t RomiDrivetrain::GetAngle() {
  return gyro.GetAngle();
}

Rotation2d RomiDrivetrain::GetRot2d() {
  return Rotation2d{GetAngle()};
}

meter_t RomiDrivetrain::GetLeftDist() {
  return length::meter_t(leftEncoder.GetDistance());
}

meter_t RomiDrivetrain::GetRightDist() {
  return length::meter_t(rightEncoder.GetDistance());
}

meter_t RomiDrivetrain::GetAveDist() {
  return (GetRightDist() + GetLeftDist()) / 2;
}

meters_per_second_t RomiDrivetrain::GetLeftSpeed() {
  return meters_per_second_t(leftEncoder.GetRate());
}

meters_per_second_t RomiDrivetrain::GetRightSpeed() {
  return meters_per_second_t(rightEncoder.GetRate());
}

meters_per_second_t RomiDrivetrain::GetAveSpeed() {
  return (GetRightSpeed() + GetLeftSpeed()) / 2;
}

void RomiDrivetrain::ArcadeDrive(double xaxisSpeed, double zaxisRotate) {
  diffDrive.ArcadeDrive(xaxisSpeed, zaxisRotate, true);
}

void RomiDrivetrain::ResetAllComponents() {
  leftEncoder.Reset();
  rightEncoder.Reset();
  gyro.Reset();
  odometry.ResetPosition(GetRot2d(), GetLeftDist(), GetRightDist(), Pose2d{});
}

Pose2d RomiDrivetrain::GetPose() {
  return currentPose;
}

void RomiDrivetrain::Periodic() {
  currentPose = odometry.Update(GetRot2d(), GetLeftDist(), GetRightDist());
}
