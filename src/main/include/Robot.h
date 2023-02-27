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
#include "AHRS.h"
#include <cameraserver/CameraServer.h>
#include "networktables/NetworkTable.h"
#include <frc/Joystick.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PowerDistribution.h>
#include <frc/Encoder.h>

#define PI 3.14159265

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
  void R2Jesu_Grid(void);
  void R2Jesu_Arm(void);
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc::Encoder armEnc{7, 8, false, frc::Encoder::k4X};
  rev::CANSparkMax armMotor{0, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder armMotorEnc = armMotor.GetEncoder();
  double armPpid = 1.25;
  double armIpid = 0.0;
  double armDpid = 0.5;
  double armPidOutput = 0.0;
  double armStops[4] = { 0.77, 0.64, 0.35, 0.26 };
  int armX;
  double armY;
  bool LAxisAllowed=true;
  frc2::PIDController m_armController{ armPpid, armIpid, armDpid, 50_ms};
  frc::DutyCycleEncoder m_encArm{9};
  frc::PowerDistribution mypdp;


  // Swerve Drive 
  frc::PS4Controller m_Drivestick{0};
  frc::PS4Controller m_Operatorstick{1};

  // Grid joystick
  frc::Joystick gridPad{2};

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

 frc::AnalogInput pressureDude{4};

double encoderConversion = (PI * 4.0) / 42.0;

  //Swerve control variables
  double conversion = 360.0/3.3;
  double conversion1 = 360.0/3.235;
  double conversion2 = 360.0/3.265;
  double conversion3 = 360.0/3.275;
  double conversion4 = 360.0/3.275;
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
  double Ppid = 0.050;
  double Ipid = 0.000;
  double Dpid = 0.001;
  double pidOutput1 = 0.0;
  double pidOutput2 = 0.0;
  double pidOutput3 = 0.0;
  double pidOutput4 = 0.0;
  frc2::PIDController m_angleController1{ Ppid , Ipid, Dpid, 100_ms};
  frc2::PIDController m_angleController2{ Ppid , Ipid, Dpid, 100_ms};
  frc2::PIDController m_angleController3{ Ppid , Ipid, Dpid, 100_ms};
  frc2::PIDController m_angleController4{ Ppid , Ipid, Dpid, 100_ms};
  double fullSpeed = .3;
  double turnSpeed = .2;
  double speedChoice;

  //Charging Station
  double switchPpid = -0.015;
  double switchIpid = 0.0;
  double switchDpid = 0.001;
  double switchPidOutput = 0.0;
  frc2::PIDController m_switchController{switchPpid, switchIpid, switchDpid, 50_ms};

  //Autonomous Turn
  double aTurnPpid = 0.02;
  double aTurnIpid = 0.0;
  double aTurnDpid = 0.001;
  double aTurnPidOutput = 0.0;
  frc2::PIDController m_aTurnController{ aTurnPpid, aTurnIpid, aTurnDpid, 20_ms};

  //Autonomous Turn 2
  double aTurn2Ppid = 0.042;
  double aTurn2Ipid = 0.00;
  double aTurn2Dpid = 0.00;
  double aTurn2PidOutput = 0.0;
  frc2::PIDController m_aTurn2Controller{ aTurn2Ppid, aTurn2Ipid, aTurn2Dpid, 20_ms};

  //April Tag Align
  double alignPpid = 0.047;
  double alignIpid = 0.001;
  double alignDpid = 0.0148;
  double alignPidOutput = 0.0;
  frc2::PIDController m_alignController{ alignPpid, alignIpid, alignDpid, 50_ms};

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
  bool p1done = false;
  double aprilID = 0.0;

  double currentDistance;
  double tid;

  int c=0;
  int v=0;
  int b=0;
  int n=0;
  int k=0;
  int j=0;
  int i=0;

  bool yDisplaceDone = false;
  bool initialBack = false;
  bool firstTurn = false;
  std::shared_ptr<nt::NetworkTable> limelight_Table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  bool alignReset = false;

  double gridPpid = 0.02;
  double gridIpid = 0.0;
  double gridDpid = 0.001;
  double gridPidOutput = 0.0;
  frc2::PIDController m_gridController{ gridPpid, gridIpid, gridDpid, 20_ms};

};
