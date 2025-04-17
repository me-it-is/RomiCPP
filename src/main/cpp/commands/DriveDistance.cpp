// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveDistance.h"
#include "units/length.h"

DriveDistance::DriveDistance(RomiDrivetrain* subsystem, units::meter_t distToDrive, int dir)
    : m_subsystem{subsystem}, m_distance{distToDrive}, dir{dir} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void DriveDistance::Initialize() {
  (*m_subsystem).ResetEncodersAndGyro();
}

void DriveDistance::Execute() {
  (*m_subsystem).ArcadeDrive(dir, 0);
}

bool DriveDistance::IsFinished() {
  return (*m_subsystem).GetAveDist() < m_distance * dir;
}