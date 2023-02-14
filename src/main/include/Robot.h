// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PS4Controller.h>
#include <frc/controller/PIDController.h>
#include <frc/motorcontrol/Spark.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/CANEncoder.h>
#include <rev/CANAnalog.h>
#include <ctre/Phoenix.h>
#include <cmath>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <chrono>
#include <thread>
#include <frc/DriverStation.h>
//#include <cscore_oo.h>
#include "AHRS.h"
#include <cameraserver/CameraServer.h>
//#include <units/angle.h>
//#include <units/length.h>
#include "networktables/NetworkTable.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  void R2Jesu_Drive(double x, double y, double z);
  void R2Jesu_SwitchAuto(void);
  void R2Jesu_Limelight(void);
  bool R2Jesu_Align(void);
  void R2Jesu_FullAuto(void);
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

    // Swerve Drive 
  frc::PS4Controller m_Drivestick{0};
  frc::PS4Controller m_Operatorstick{1};
  
  //swerve 1 front left
  rev::CANSparkMax m_SwerveDrive1{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder1 = m_SwerveDrive1.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn1{1};
  frc::AnalogInput m_SwerveAnalog1{0};
  
  //swerve 2
  rev::CANSparkMax m_SwerveDrive2{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder2 = m_SwerveDrive2.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn2{2};
  frc::AnalogInput m_SwerveAnalog2{1};
  
  //swerve 3
  rev::CANSparkMax m_SwerveDrive3{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder3 = m_SwerveDrive3.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn3{3};
  frc::AnalogInput m_SwerveAnalog3{2};
  
  //swerve 4
  rev::CANSparkMax m_SwerveDrive4{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder4 = m_SwerveDrive4.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn4{4};
  frc::AnalogInput m_SwerveAnalog4{3};

  //Swerve control variables
  double conversion = 360.0/3.3;
  double conversion1 = 360.0/3.235;
  double conversion2 = 360.0/3.265;
  double conversion3 = 360.0/3.275;
  double conversion4 = 360.0/3.275;
  //double x=0.0, y=0.0, z=0.0;
  double inputAngle = 0.0;
  double r = 0.0;
  double newX = 0.0, newY = 0.0;
  double fieldOrientedAngle = 0.0;
  double correctionPID;
  double LENGTH = 17.375;
  double WIDTH = 21.25;
  double A=0.0;
	double B=0.0;
	double C=0.0;
	double D=0.0;
  double wSpeed1=0.0;
  double wAngle1=0.0;
  double wSpeed2=0.0;
  double wAngle2=0.0;
  double wSpeed3=0.0;
  double wAngle3=0.0;
  double wSpeed4=0.0;
  double wAngle4=0.0;
  double R = sqrt((LENGTH*LENGTH) + (WIDTH*WIDTH));
  double Ppid = 0.050;//45;
  double Ipid = 0.000;
  double Dpid = 0.001;//0.0008;//.0005
  double switchPpid = -0.010;//maura we added a zero 0.025
  double switchIpid = 0.0;
  double switchDpid = 0.0;
  double pidOutput1 = 0.0;
  double pidOutput2 = 0.0;
  double pidOutput3 = 0.0;
  double pidOutput4 = 0.0;
  double switchPidOutput = 0.0;
  frc2::PIDController m_angleController1{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_angleController2{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_angleController3{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_angleController4{ Ppid , Ipid, Dpid, 20_ms};
  frc2::PIDController m_switchController{ switchPpid, switchIpid, switchDpid, 50_ms};
  double fullSpeed = .3;
  double turnSpeed = .2;
  double speedChoice;

  //NavX
  AHRS *ahrs;
  
  //Align
  double facingError;
  double facingCorrection;
  double alignYaw;
  double aprilError;
  double aprilCorrection;
  bool hasRun = false;
  bool hasRunDistance1 = false;
  bool hasRunDistance2 = false;
  bool hasRunAngle1 = false;
  double aprilID = 0.0;

  std::shared_ptr<nt::NetworkTable> limelight_Table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

};
