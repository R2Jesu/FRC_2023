#include "Robot.h"

int loopy;

void Robot::R2Jesu_Arm()
{
  frc::SmartDashboard::PutNumber("fullSpeed", fullSpeed);
  if (fullSpeed <= .2 || frc::DriverStation::IsAutonomousEnabled()) {
     if (gridPad.GetRawButton(8)) {
         if (armX > 1) 
         {
           m_armController.SetPID(upPpid, upIpid, upDpid);
         }
         else if (armX < 1)
         {
           m_armController.SetPID(downPpid, downIpid, downDpid);
         }
   
         armX=1;
         armSetPoint=armStops[armX];
     }
     if (gridPad.GetRawButton(7) || gridPad.GetRawButton(9)) {
       if (gridPad.GetRawButton(9)) {
         if (armSetPoint < 0.91) {
            armSetPoint=armSetPoint + 0.0003;
            m_armController.Reset();
            m_armController.SetPID(upPpid, upIpid, upDpid);
         }
       }
       else {
         if (armSetPoint > .1) {
            armSetPoint=armSetPoint - 0.0003;
            m_armController.Reset();
            m_armController.SetPID(downPpid, downIpid, downDpid);
         }
       }
       for (loopy=0;loopy < (sizeof(armStops) / sizeof(double)); loopy++)
       {
         if (armSetPoint > armStops[loopy])
         {
           armX=loopy;
           break;
         }
       }
     }
     else {
       if (m_Operatorstick.GetTriangleButtonPressed()) {
         if (armX > 0) {
           armX--;
           armSetPoint=armStops[armX];
           m_armController.SetPID(upPpid, upIpid, upDpid);
         }
       }
       if (m_Operatorstick.GetCrossButtonPressed()) {
         if (armX < ((sizeof(armStops) / sizeof(double)) - 1)) {
             armX++;
             armSetPoint=armStops[armX];
             m_armController.SetPID(downPpid, downIpid, downDpid);
           }
       }
       if (gridPad.GetRawButtonPressed(1) || gridPad.GetRawButtonPressed(2) || gridPad.GetRawButtonPressed(3)) {
         if (armX > 3) 
         {
           m_armController.SetPID(upPpid, upIpid, upDpid);
         }
         else if (armX < 3)
         {
           m_armController.SetPID(downPpid, downIpid, downDpid);
         }
   
         armX=3;
         armSetPoint=armStops[armX];
       }
       if (gridPad.GetRawButtonPressed(4) || gridPad.GetRawButtonPressed(5) || gridPad.GetRawButtonPressed(6)) {
         if (armX > 2) 
         {
           m_armController.SetPID(upPpid, upIpid, upDpid);
         }
         else if (armX < 2)
         {
           m_armController.SetPID(downPpid, downIpid, downDpid);
         }
         armX=2;
         armSetPoint=armStops[armX];
       }
   
     }
     armPidOutput = m_armController.Calculate((m_encArm.GetAbsolutePosition()), armSetPoint);
     if (((armX == 0) || (armSetPoint == 0.91)) && !(frc::DriverStation::IsAutonomousEnabled()))
     {
       armPidOutput = armPidOutput * .5;
     }
     if (armX == 1)
     {
       armPidOutput = armPidOutput * .5;
     }
   
     armMotor.Set(armPidOutput);
  }
}