#include "Robot.h"

void Robot::R2Jesu_FullAuto()
{

    //TODO: What tag are we going to face?
    if (tid != 8.0 && !(p1done) && !(firstTurn))
    {
        frc::SmartDashboard::PutNumber("turn",c++);
        alignYaw = ahrs->GetYaw();
        aTurnPidOutput = m_aTurnController.Calculate(alignYaw, 180);
        R2Jesu_Drive(0.0, 0.0, aTurnPidOutput);
    } else
    {
        if (tid == 8.0 && !(firstTurn) && !(p1done))
        {
            firstTurn = true;
            R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }

    if (!(p1done)) {

        //TODO: What tag are we going to face?
        if (R2Jesu_Align() && currentDistance > 23.0 && tid == 8.0)
        {
            R2Jesu_Drive(0.0, -0.4, 0.0);
            frc::SmartDashboard::PutNumber("forward",b++);
        }
        else
        {
            //TODO: What tag are we going to face?
            if (currentDistance < 23.0 && tid == 8.0) 
            {
                R2Jesu_Drive(0.0, 0.0, 0.0);
                p1done = true;
                m_DriveEncoder1.SetPosition(0.0);
            }
        }
    }
    frc::SmartDashboard::PutNumber("aTurnPidOutput", aTurnPidOutput);
    frc::SmartDashboard::PutNumber("tid", limelight_Table->GetNumber("tid",0.0));
    tid = limelight_Table->GetNumber("tid", 0.0);
    frc::SmartDashboard::PutBoolean("p1done", p1done);
    frc::SmartDashboard::PutBoolean("firstturn", firstTurn);
}