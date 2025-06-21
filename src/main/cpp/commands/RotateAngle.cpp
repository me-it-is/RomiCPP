// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAngle.h"
#include "units/angle.h"

RotateAngle::RotateAngle(RomiDrivetrain* subsystem, units::radian_t angleToRotate, int dir)
    : m_subsystem{subsystem}, m_angle{angleToRotate}, dir{dir} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void RotateAngle::Initialize() {
  (*m_subsystem).ResetEncodersAndGyro();
}

void RotateAngle::Execute() {
  (*m_subsystem).ArcadeDrive(0, dir);
}

bool RotateAngle::IsFinished() {
  return (*m_subsystem).GetAngle() < m_angle * dir;
}