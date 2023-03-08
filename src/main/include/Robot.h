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
  bool R2Jesu_Align(double targetYaw);
  void R2Jesu_FullAuto(void);
  void R2Jesu_Grid(void);
  void R2Jesu_Arm(void);
  void R2Jesu_Chewy(void);
  frc::SendableChooser<bool> m_chargeChoice;
  const bool kAutoRun = true;
  const bool kAutoNoRun = false;
  bool m_chargeSelected;
  frc::SendableChooser<double> m_tagChoice;
  const double april1 = 1.0;
  const double april2 = 2.0;
  const double april3 = 3.0;
  const double april6 = 6.0;
  const double april7 = 7.0;
  const double april8 = 8.0;
  double m_aprilSelected;
  frc::SendableChooser<double> m_gridChoice;
  const double grid1 = 1.0;
  const double grid2 = 2.0;
  const double grid3 = 3.0;
  const double grid4 = 4.0;
  const double grid5 = 5.0;
  const double grid6 = 6.0;
  double m_gridSelected;




  // Arm
  frc::DutyCycleEncoder m_encArm{0};
  rev::CANSparkMax armMotor{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder armMotorEnc = armMotor.GetEncoder();
  double armPpid = 5.0; //1.5
  double armIpid = 0.00; //0
  double armDpid = 0.0; //.5
  double armPidOutput = 0.0;
  double armStops[5] = { 0.91, 0.88, 0.35, 0.305, 0.115};
  int armX;
  double armY;
  double armSetPoint = 0.0;
  bool runToSet=true;
  frc2::PIDController m_armController{ armPpid, armIpid, armDpid, 20_ms};
  frc::PowerDistribution mypdp;
  double upPpid = 8.0;
  double upIpid = 0.0;//0.3;
  double upDpid = 0.0; //0.5
  double downPpid = 1.4;//1.3;
  double downIpid = 0.0;//0.27;
  double downDpid = 0.0; //0.55
  // Chewy
    rev::CANSparkMax chewyMotor{10, rev::CANSparkMax::MotorType::kBrushed};

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
  //ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn3{3};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_SwerveTurn3{3};
  frc::AnalogInput m_SwerveAnalog3{2};
  
  //swerve 4
  rev::CANSparkMax m_SwerveDrive4{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder4 = m_SwerveDrive4.GetEncoder();
  //ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn4{4};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_SwerveTurn4{4};
  frc::AnalogInput m_SwerveAnalog4{3};

 frc::AnalogInput pressureDude{4};

 frc::AnalogInput stringDude{5};

double encoderConversion = (PI * 4.0) / 42.0;

  //Swerve control variables
  double conversion = 360.0/5.0;
  double conversion1 = 360.0/3.235;
  double conversion2 = 360.0/3.265;
  double conversion3 = 360.0/3.275;
  double conversion4 = 360.0/3.275;
  double inputAngle = 0.0;
  double r = 0.0;
  double newX = 0.0, newY = 0.0;
  double fieldOrientedAngle = 0.0;
  double correctionPID;
  double LENGTH = 20.0;
  double WIDTH = 17.5;
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
  double fullSpeed;
  double turnSpeed = .15;
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
  double alignPpid = 0.047; //.047
  double alignIpid = 0.005; // .001
  double alignDpid = 0.0148; // .0148
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

  bool yDisplaceDone = false;
  bool initialBack = false;
  double initialBackDistance = 0.0;
  bool turnt = false;
  bool firstTurn = false;
  std::shared_ptr<nt::NetworkTable> limelight_Table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  bool alignReset = false;

  double gridPpid = 0.015;
  double gridIpid = 0.0;
  double gridDpid = 0.001;
  double gridPidOutput = 0.0;
  frc2::PIDController m_gridController{gridPpid, gridIpid, gridDpid, 20_ms};

  bool fullAuto1 = false;
  bool fullAuto2 = false;
  bool fullAuto2_5 = false;
  bool fullAuto3 = false;
  bool fullAuto4 = false;
  bool fullAuto5 = false;
  bool fullAuto6 = false;
  double autoAprilDistance = 0.0;
  double autoArmSet = 0.0;
  double autoOffset = 0.0;

  bool wheelAngleCheck = false;

  //camera
  cs::UsbCamera drvCamera;
};
