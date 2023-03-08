#include "Robot.h"

double opcl;

void Robot::R2Jesu_Chewy()
{

  opcl = 0.0;
  if (m_Operatorstick.GetR1Button() && (stringDude.GetVoltage() > 2.4)) {
    printf("R1\n");
    opcl = -0.5;
  }
  if (m_Operatorstick.GetL2Axis() && !(pressureDude.GetVoltage() >= 2.25)) {
    printf("L2 axis\n");
    opcl = 0.8;
  }
  if (((armX == 0) || (armSetPoint == 0.91)) && !(frc::DriverStation::IsAutonomousEnabled()))
  {
    opcl = 0.4;
  }
  chewyMotor.Set(opcl);

}