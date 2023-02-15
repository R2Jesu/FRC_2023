// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  ahrs = new AHRS(frc::SPI::Port::kMXP);
  m_angleController1.EnableContinuousInput(0.00, 360.00);
  m_angleController2.EnableContinuousInput(0.00, 360.00);
  m_angleController3.EnableContinuousInput(0.00, 360.00);
  m_angleController4.EnableContinuousInput(0.00, 360.00);

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  R2Jesu_Limelight();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  ahrs->ResetDisplacement();
  //m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  m_DriveEncoder1.SetPosition(0.0);
  m_DriveEncoder2.SetPosition(0.0);
  m_DriveEncoder3.SetPosition(0.0);
  m_DriveEncoder4.SetPosition(0.0);
  m_SwerveDrive1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive3.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive4.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  /*if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/
  //R2Jesu_FullAuto();

}

void Robot::AutonomousPeriodic() {
  R2Jesu_FullAuto();
  //R2Jesu_SwitchAuto();
  /*if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/
}

void Robot::TeleopInit() 
{
  m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
  m_SwerveDrive1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_SwerveDrive2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_SwerveDrive3.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_SwerveDrive4.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::TeleopPeriodic() 
{
  R2Jesu_Drive(m_Drivestick.GetR2Axis(), m_Drivestick.GetRightY() * -1.0, m_Drivestick.GetLeftX());
  if (m_Drivestick.GetTriangleButton())
  {
    R2Jesu_Align();
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
