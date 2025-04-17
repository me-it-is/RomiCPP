// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/RomiDrivetrain.h"
#include "commands/DriveCommand.h"
#include "commands/DriveDistance.h"
#include "commands/RotateAngle.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc::Joystick m_rightJoystick{OperatorConstants::kDriverControllerPort};
  // The robot's subsystems are defined here...
  RomiDrivetrain m_subsystem;

  DriveCommand driveCommand{&m_subsystem, &m_rightJoystick};
  DriveDistance driveDistance{&m_subsystem, 1_m, 1};
  RotateAngle rotateAngle{&m_subsystem, 1_rad, 1};

  void ConfigureBindings();
};
