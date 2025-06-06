// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "subsystems/RomiDrivetrain.h"
#include "units/length.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveCommand
    : public frc2::CommandHelper<frc2::Command, DriveCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit DriveCommand(RomiDrivetrain* subsystem, frc::Joystick* joystick);

  void Execute() override;

  bool IsFinished() override;

 private:
  RomiDrivetrain* m_subsystem;
  frc::Joystick* m_joystick;
};
