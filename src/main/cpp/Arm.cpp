#include "Robot.h"


void Robot::R2Jesu_Arm()
{
    armY = m_Operatorstick.GetRightY();
  frc::SmartDashboard::PutNumber("y",armY);
  armMotor.Set(armY);

  if ((armY>-0.1) && (armY<0.1)) {
 /*  armPidOutput = m_armController.Calculate((m_encArm.GetAbsolutePosition()), 0.64);
  armMotor.Set(armPidOutput); */
  if (m_Operatorstick.GetR1ButtonPressed()) {
    if (armX > 0) {
      armX--;
    }
  }
  if (m_Operatorstick.GetL2Axis() && LAxisAllowed) {
    LAxisAllowed=false;
    if (armX < ((sizeof(armStops) / sizeof(double)) - 1)) {
        armX++;
      }
    }

  armPidOutput = m_armController.Calculate((m_encArm.GetAbsolutePosition()), armStops[armX]);
  armMotor.Set(armPidOutput);
  if (!m_Operatorstick.GetL2Axis()) {
    LAxisAllowed=true;
  }

  frc::SmartDashboard::PutNumber("armPidOutput",armPidOutput);
  frc::SmartDashboard::PutNumber("current position",m_encArm.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("x",armX);
 
  //m_encArm.GetAbsolutePosition();z
  }
   frc::SmartDashboard::PutNumber("current",mypdp.GetCurrent(3));
}