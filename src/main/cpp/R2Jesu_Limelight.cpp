#include "Robot.h"

void Robot::R2Jesu_Limelight()
{

   //double targetOffsetAngle_Horizontal = limelight_Table->GetNumber("tx",0.0);
   double targetOffsetAngle_Vertical = limelight_Table->GetNumber("ty",0.0);
   //double targetArea = limelight_Table->GetNumber("ta",0.0);
   //double targetSkew = limelight_Table->GetNumber("ts",0.0);
   frc::SmartDashboard::PutNumber("targetOffsetAngle_Horizontal", limelight_Table->GetNumber("tx",0.0));
   frc::SmartDashboard::PutNumber("targetOffsetAngle_Vertical", limelight_Table->GetNumber("ty",0.0));
   frc::SmartDashboard::PutNumber("targetArea", limelight_Table->GetNumber("ta",0.0));
   frc::SmartDashboard::PutNumber("targetSkew", limelight_Table->GetNumber("ts",0.0));
 

   //(Target height - camera height) / tan((camera angle + target offset angle from limelight)) * (PI / 180)))
   double ourDist = (((double)18.0 - (double)13.5) / (tan(((double)0.86 + targetOffsetAngle_Vertical) * (PI / 180.00))));
   currentDistance = ourDist;
   if (targetOffsetAngle_Vertical == 0.0){
      currentDistance = 0.0;
   }
   frc::SmartDashboard::PutNumber("current distance", currentDistance);
}