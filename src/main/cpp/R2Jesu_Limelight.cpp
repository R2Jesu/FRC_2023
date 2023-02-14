#include "Robot.h"

void Robot::R2Jesu_Limelight()
{

   double targetOffsetAngle_Horizontal = limelight_Table->GetNumber("tx",0.0);
   double targetOffsetAngle_Vertical = limelight_Table->GetNumber("ty",0.0);
   double targetArea = limelight_Table->GetNumber("ta",0.0);
   double targetSkew = limelight_Table->GetNumber("ts",0.0);
   frc::SmartDashboard::PutNumber("targetOffsetAngle_Horizontal", limelight_Table->GetNumber("tx",0.0));
   frc::SmartDashboard::PutNumber("targetOffsetAngle_Vertical", limelight_Table->GetNumber("ty",0.0));
   frc::SmartDashboard::PutNumber("targetArea", limelight_Table->GetNumber("ta",0.0));
   frc::SmartDashboard::PutNumber("targetSkew", limelight_Table->GetNumber("ts",0.0));
 
}