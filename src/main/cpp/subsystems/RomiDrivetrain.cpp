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
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPLTVController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>

using namespace pathplanner;
using namespace frc;
using namespace units;

RomiDrivetrain::RomiDrivetrain() {
  leftEncoder.SetDistancePerPulse(DrivetrainConstants::kDistPerPulseDouble);
  rightEncoder.SetDistancePerPulse(DrivetrainConstants::kDistPerPulseDouble);

  rightSpark.SetInverted(true);

  ResetAllComponents();

  RobotConfig config = RobotConfig::fromGUISettings();

  // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](const frc::Pose2d &pose){ ResetPoseTo(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](const frc::ChassisSpeeds &speeds){ DriveWithChassisSpeeds(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPLTVController>(0.02_s), // PPLTVController is the built in path following controller for differential drive trains
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

radian_t RomiDrivetrain::GetAngle() {
  return gyro.GetAngle();
}

Rotation2d RomiDrivetrain::GetRot2d() {
  return Rotation2d{GetAngle()};
}

radians_per_second_t RomiDrivetrain::GetAngularVelocity() {
  return gyro.GetRate();
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

void RomiDrivetrain::ResetPoseTo(Pose2d pose) {
  odometry.ResetPosition(GetRot2d(), GetLeftDist(), GetRightDist(), pose);
}

ChassisSpeeds RomiDrivetrain::GetRobotRelativeSpeeds() {
  return ChassisSpeeds::Discretize(GetAveSpeed(), 0_mps, GetAngularVelocity(), 0.02_s);
}

void RomiDrivetrain::DriveWithChassisSpeeds(ChassisSpeeds speeds) {
  ArcadeDrive((speeds.vx / DrivetrainConstants::kMaxLinearVelocity).value(), (speeds.omega / DrivetrainConstants::kMaxAngularVelocity).value());
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
