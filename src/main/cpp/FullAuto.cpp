#include "Robot.h"

void Robot::R2Jesu_FullAuto()
{
    if (limelight_Table->GetNumber("tid", 0) != 8)
    {
        printf("in turn\n");
        alignYaw = ahrs->GetYaw();
        aTurnPidOutput = m_aTurnController.Calculate(alignYaw, 180);
        //facingError = (fabs(alignYaw) - 180) * (fabs(alignYaw) / alignYaw);
        //facingCorrection = -.02 * facingError;
        //R2Jesu_Drive(0.0, 0.0, facingCorrection);
        frc::SmartDashboard::PutNumber("facingCorrection", aTurnPidOutput);
        R2Jesu_Drive(0.0, 0.0, aTurnPidOutput);
    }
    R2Jesu_Drive(0.0, 0.0, 0.0);
    //while(R2Jesu_Align() == false);
    //m_DriveEncoder1.SetPosition(0.0);
    // while (m_DriveEncoder1.GetPosition() < 20.0 && frc::DriverStation::IsAutonomousEnabled())
    if (R2Jesu_Align() == true && currentDistance > 23.0)
    {
        R2Jesu_Drive(0.0, -0.4, 0.0);
        printf("driving forward\n");
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