// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  //camera
  drvCamera = frc::CameraServer::StartAutomaticCapture();
  drvCamera.SetResolution(320, 240);
  drvCamera.SetFPS(15);
  drvCamera.SetExposureManual(40);


  armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_chargeChoice.SetDefaultOption("Run Charge", kAutoRun);
  m_chargeChoice.AddOption("No Charge", kAutoNoRun);
  frc::SmartDashboard::PutData("Charger Choice", &m_chargeChoice);
  m_tagChoice.SetDefaultOption("April 1", april1); 
  m_tagChoice.AddOption("April 2", april2);
  m_tagChoice.AddOption("April 3", april3);
  m_tagChoice.AddOption("April 6", april6);
  m_tagChoice.AddOption("April 6", april6);
  m_tagChoice.AddOption("April 7", april7);
  m_tagChoice.AddOption("April 8", april8);
  frc::SmartDashboard::PutData("April Choice", &m_tagChoice);
  m_gridChoice.SetDefaultOption("Grid 1", grid1); 
  m_gridChoice.AddOption("Grid 2", grid2);
  m_gridChoice.AddOption("Grid 3", grid3);
  m_gridChoice.AddOption("Grid 4", grid4);
  m_gridChoice.AddOption("Grid 5", grid5);
  m_gridChoice.AddOption("Grid 6", grid6);
  frc::SmartDashboard::PutData("Grid Choice", &m_gridChoice);

  ahrs = new AHRS(frc::SPI::Port::kMXP);
  m_angleController1.EnableContinuousInput(0.00, 360.00);
  m_angleController2.EnableContinuousInput(0.00, 360.00);
  m_angleController3.EnableContinuousInput(0.00, 360.00);
  m_angleController4.EnableContinuousInput(0.00, 360.00);
  m_DriveEncoder1.SetPosition(0.0);
  m_DriveEncoder1.SetPositionConversionFactor(1.795);
  armMotor.Set(0.0);
  armX = 0;
  armSetPoint = 0.91;
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
  frc::SmartDashboard::PutNumber("encoder x", m_DriveEncoder1.GetPosition());
  R2Jesu_Limelight();
  frc::SmartDashboard::PutNumber("Pressure voltage", pressureDude.GetVoltage());
  frc::SmartDashboard::PutNumber("String Voltage", stringDude.GetVoltage());
  frc::SmartDashboard::PutNumber("Auto April Distance", autoAprilDistance);
  /*if (m_Operatorstick.GetCircleButtonPressed())
  {
    printf("Circle Button\n");
  }
  if (m_Operatorstick.GetCrossButtonPressed())
  {
    printf("Cross Button\n");
  }
  if (m_Operatorstick.GetL1ButtonPressed())
  {
    printf("L1Button Button\n");
  }
  if (m_Operatorstick.GetL2Axis())
  {
    printf("L2Axis Button\n");
  }
  if (m_Operatorstick.GetL2ButtonPressed())
  {
    printf("L2 Button\n");
  }
  if (m_Operatorstick.GetL3ButtonPressed())
  {
    printf("L3 Button\n");
  }
  if (m_Operatorstick.GetOptionsButtonPressed())
  {
    printf("Options Button\n");
  }
  if (m_Operatorstick.GetPSButtonPressed())
  {
    printf("PS Button\n");
  }
  if (m_Operatorstick.GetR1ButtonPressed())
  {
    printf("R1 Button\n");
  }
  if (m_Operatorstick.GetR2Axis())
  {
    printf("R2Axis Button\n");
  }
  if (m_Operatorstick.GetR2ButtonPressed())
  {
    printf("R2 Button\n");
  }
  if (m_Operatorstick.GetR3ButtonPressed())
  {
    printf("R3 Button\n");
  }
  if (m_Operatorstick.GetShareButtonPressed())
  {
    printf("Share Button\n");
  }
  if (m_Operatorstick.GetSquareButtonPressed())
  {
    printf("Square Button\n");
  }
  if (m_Operatorstick.GetTriangleButtonPressed())
  {
    printf("Triangle Button\n");
  }
  if (m_Operatorstick.GetTouchpad())
  {
    printf("Touchpad\n");
  }*/
  frc::SmartDashboard::PutBoolean("P1 Done", p1done);
  frc::SmartDashboard::PutNumber("Grid pid output", gridPidOutput);
  frc::SmartDashboard::PutNumber("arm PID", armPidOutput);
  frc::SmartDashboard::PutNumber("Arm Encoder", m_encArm.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("set point", armSetPoint);
  frc::SmartDashboard::PutNumber("Arm X", armX);
  frc::SmartDashboard::PutNumber("encoder y", m_DriveEncoder1.GetPosition());
  frc::SmartDashboard::PutNumber("encoder x", m_DriveEncoder1.GetPosition() * encoderConversion);
  frc::SmartDashboard::PutNumber("Pitch", ahrs->GetPitch());
  frc::SmartDashboard::PutNumber("switch pid", switchPidOutput);
  frc::SmartDashboard::PutBoolean("initalback", initialBack);
  frc::SmartDashboard::PutBoolean("yDisplaceDone", yDisplaceDone);
  frc::SmartDashboard::PutNumber("position3.2", m_DriveEncoder3.GetPosition());
  frc::SmartDashboard::PutNumber("tid", limelight_Table->GetNumber("tid",0.0));
  frc::SmartDashboard::PutBoolean("Full Auto 1", fullAuto1);
  frc::SmartDashboard::PutBoolean("Full Auto 2", fullAuto2);
  frc::SmartDashboard::PutBoolean("Full Auto 2_5", fullAuto2_5);
  frc::SmartDashboard::PutBoolean("Full Auto 3", fullAuto3);
  frc::SmartDashboard::PutBoolean("Full Auto 4", fullAuto4);
  frc::SmartDashboard::PutBoolean("Full Auto 5", fullAuto5);
  frc::SmartDashboard::PutBoolean("Full Auto 6", fullAuto6);
  frc::SmartDashboard::PutBoolean("turnt", turnt);
  frc::SmartDashboard::PutNumber("YAW", ahrs->GetYaw());
  frc::SmartDashboard::PutBoolean("hasRun", hasRun);
  frc::SmartDashboard::PutNumber("tx", limelight_Table->GetNumber("tx",0.0));
  frc::SmartDashboard::PutNumber("aTurnPidOutput", aTurnPidOutput);
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
  fullSpeed = 0.5;
  fullAuto1 = false;
  fullAuto2 = false;
  fullAuto2_5 = false;
  fullAuto3 = false;
  fullAuto4 = false;
  fullAuto5 = false;
  fullAuto6 = false;
  armSetPoint = 0.91;
  pidOutput1 = 0.0;
  pidOutput2 = 0.0;
  pidOutput3 = 0.0;
  pidOutput4 = 0.0;
  speedChoice = 0.0;
  aTurnPidOutput = 0.0;
  aTurn2PidOutput = 0.0;
  alignPidOutput = 0.0;
  ahrs->ResetDisplacement();
  m_chargeSelected = m_chargeChoice.GetSelected();
  m_aprilSelected = m_tagChoice.GetSelected();
  m_gridSelected = m_gridChoice.GetSelected();
  m_DriveEncoder1.SetPosition(0.0);
  m_DriveEncoder2.SetPosition(0.0);
  m_DriveEncoder3.SetPosition(0.0);
  m_DriveEncoder4.SetPosition(0.0);
  m_SwerveDrive1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive3.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_SwerveDrive4.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  p1done = false;
  hasRun = false;
  yDisplaceDone = false;
  initialBack = false;
  turnt = false;
  firstTurn = false;
  hasRun = false;
  hasRunDistance1 = false;
  hasRunDistance2 = false;
  hasRunAngle1 = false;
  p1done = false;
  inputAngle = 0.0;
  turnSpeed = .15;
  if ((m_gridSelected == 1.0) || (m_gridSelected == 2.0) || (m_gridSelected == 3.0))
  {
    autoAprilDistance = 41.0;
    autoArmSet = 0.35;
    initialBackDistance = -0.0;
  } else
  {
    autoAprilDistance = 25.0;
    autoArmSet = 0.35;
    initialBackDistance = -12.0;
  }
  
  if ((m_gridSelected == 1.0) || (m_gridSelected == 4.0))
  {
    autoOffset = 36.5;
  }
  if ((m_gridSelected == 3.0) || (m_gridSelected == 6.0))
  {
    autoOffset = -36.5;
  }
}

void Robot::AutonomousPeriodic() {

  if((fullAuto6 == false))
  {
    R2Jesu_FullAuto();
  }
  if (fullAuto6 && m_chargeSelected)
  {
    R2Jesu_SwitchAuto();
  }

  R2Jesu_Arm();
}

void Robot::TeleopInit() 
{
  armSetPoint = 0.91;
  inputAngle = 0.0;
  speedChoice = 0.0;
  m_DriveEncoder1.SetPosition(0.0);
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
  armMotor.Set(0.0);
  pidOutput1 = 0.0;
  pidOutput2 = 0.0;
  pidOutput3 = 0.0;
  pidOutput4 = 0.0;
  aTurnPidOutput = 0.0;
  aTurn2PidOutput = 0.0;
  alignPidOutput = 0.0;
  hasRun = false;
  hasRunDistance1 = false;
  hasRunDistance2 = false;
  hasRunAngle1 = false;
  p1done = false;
  turnSpeed = .15;
}

void Robot::TeleopPeriodic() 
{
  if (armX < 2) {
    fullSpeed = .2 + (.5 * m_Drivestick.GetL2Axis());
  }  
  R2Jesu_Drive(m_Drivestick.GetR2Axis(), m_Drivestick.GetRightY() * -1.0, m_Drivestick.GetLeftX());
  // triangle is y
  if (m_Drivestick.GetTriangleButton())
  {
    if (abs(ahrs->GetYaw()) < 90) {
       alignReset = R2Jesu_Align(0.0);
    }
    else {
       alignReset = R2Jesu_Align(180.0);
    }
  }
  if (alignReset)
  {
    m_DriveEncoder1.SetPosition(0.0);
    alignReset = false;
  }
  if ((fabs(m_Drivestick.GetRightX()) < 0.1) && (fabs(m_Drivestick.GetRightY()) < 0.1) && (fabs(m_Drivestick.GetLeftX()) < 0.1))
  {
    R2Jesu_Grid();
  }
  R2Jesu_Arm();
  R2Jesu_Chewy();
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
