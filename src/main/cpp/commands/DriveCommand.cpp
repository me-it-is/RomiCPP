// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommand.h"
#include <frc/Joystick.h>
#include "units/length.h"

DriveCommand::DriveCommand(RomiDrivetrain* subsystem, frc::Joystick* joystick)
    : m_subsystem{subsystem}, m_joystick{joystick} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void DriveCommand::Execute() {
  (*m_subsystem).ArcadeDrive((*m_joystick).GetX(), (*m_joystick).GetY());
}

bool DriveCommand::IsFinished() {
  return false;
}