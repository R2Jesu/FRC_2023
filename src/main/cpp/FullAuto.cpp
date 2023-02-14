#include "Robot.h"

void Robot::R2Jesu_FullAuto()
{
    while ((ahrs->GetYaw() > -178.5) && (ahrs->GetYaw() < 178.5))
    {
        printf("in turn\n");
        alignYaw = ahrs->GetYaw();
        facingError = (fabs(alignYaw) - 180) * (fabs(alignYaw) / alignYaw);
        facingCorrection = -.02 * facingError;
        //R2Jesu_Drive(0.0, 0.0, facingCorrection);
        frc::SmartDashboard::PutNumber("facingCorrection", facingCorrection);
        R2Jesu_Drive(0.0, 0.0, facingCorrection);
    }
    while(R2Jesu_Align() == false);
    m_DriveEncoder1.SetPosition(0.0);
    while (m_DriveEncoder1.GetPosition() < 20.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        R2Jesu_Drive(0.0, -0.4, 0.0);
    }
    //while(!(R2Jesu_Align()));
    /*aprilID = limelight_Table->GetNumber("id", 0.0);
    m_DriveEncoder1.SetPosition(0.0);
    if ((aprilID == 8.0) || (aprilID == 3.0))
    {
        while (m_DriveEncoder1.GetPosition() < 66.0 && frc::DriverStation::IsAutonomousEnabled())
        {
            R2Jesu_Drive(-0.4, 0.0, 0.0);
        }
    }  else if ((aprilID == 6.0) || (aprilID == 1.0))
    {
        while (m_DriveEncoder1.GetPosition() < 66.0 && frc::DriverStation::IsAutonomousEnabled())
        {
            R2Jesu_Drive(0.4, 0.0, 0.0);
        }
    }*/
}