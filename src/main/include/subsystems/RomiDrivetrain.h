// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"
#include "frc/motorcontrol/Spark.h"
#include "frc/motorcontrol/MotorController.h"
#include "frc/Encoder.h"
#include "frc/romi/RomiGyro.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/geometry/Rotation2d.h"
#include <frc/geometry/Pose2d.h>
#include "units/base.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>

using meter_t = units::meter_t;
using meters_per_second_t = units::meters_per_second_t;

class RomiDrivetrain : public frc2::SubsystemBase {
 public:
  RomiDrivetrain();

  /**
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  units::radian_t GetAngle();
  frc::Rotation2d GetRot2d();
  units::radians_per_second_t GetAngularVelocity();

  meter_t GetLeftDist();
  meter_t GetRightDist();
  meter_t GetAveDist();
  meters_per_second_t GetLeftSpeed();
  meters_per_second_t GetRightSpeed();
  meters_per_second_t GetAveSpeed();

  void ArcadeDrive(double xaxisSpeed, double zaxisRotate);

  void ResetPoseTo(frc::Pose2d pose);
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  void DriveWithChassisSpeeds(frc::ChassisSpeeds speeds);

  void ResetAllComponents();

  frc::Pose2d GetPose();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  frc::Spark leftSpark{0};
  frc::Spark rightSpark{1};
  frc::Encoder leftEncoder{4, 5 , false};
  frc::Encoder rightEncoder{6, 7 , false};

  frc::RomiGyro gyro{};
  frc::DifferentialDrive diffDrive{leftSpark, rightSpark};
  frc::DifferentialDriveKinematics kinematics{149_mm};
  frc::DifferentialDriveOdometry odometry{frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{}};

  frc::Pose2d currentPose;
};
