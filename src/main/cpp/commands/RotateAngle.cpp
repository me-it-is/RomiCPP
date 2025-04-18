// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAngle.h"
#include "units/angle.h"

using namespace frc;
using namespace units;

RotateAngle::RotateAngle(RomiDrivetrain* subsystem, radian_t angleToRotate, int dir, PIDController* PIDController)
    : m_subsystem{subsystem}, m_angle{angleToRotate}, dir{dir}, controller{PIDController} {
  (*controller).SetSetpoint(m_angle.value());
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void RotateAngle::Initialize() {
  (*m_subsystem).ResetAllComponents();
}

void RotateAngle::Execute() {
  double output = (*controller).Calculate((*m_subsystem).GetAngle().value());
  (*m_subsystem).ArcadeDrive(0, dir);
}

bool RotateAngle::IsFinished() {
  return (*controller).AtSetpoint();
}