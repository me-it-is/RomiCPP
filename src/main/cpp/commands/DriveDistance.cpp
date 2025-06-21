// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveDistance.h"
#include "units/length.h"

DriveDistance::DriveDistance(RomiDrivetrain* subsystem, units::meter_t distToDrive, int dir, frc::PIDController* PIDController)
    : m_subsystem{subsystem}, m_distance{distToDrive}, dir{dir}, controller{PIDController} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void DriveDistance::Initialize() {
  (*m_subsystem).ResetAllComponents();
  (*controller).SetSetpoint(m_distance.value());
}

void DriveDistance::Execute() {
  double output = (*controller).Calculate((*m_subsystem).GetAveDist().value());
  (*m_subsystem).ArcadeDrive(output * dir, 0);
}

bool DriveDistance::IsFinished() {
  return (*controller).AtSetpoint();
}