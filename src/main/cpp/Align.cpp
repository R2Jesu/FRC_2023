#include "Robot.h"
//bool localAlign;

bool Robot::R2Jesu_Align(double targetYaw)
{
    localAlign = true;
    aprilCorrection = 0.0;
    aTurn2PidOutput = 0.0;
    //if ((ahrs->GetYaw() > -177.0) && (ahrs->GetYaw() < 177.0))
    frc::SmartDashboard::PutNumber("targetYaw", targetYaw);
    if (abs(ahrs->GetYaw()) < (targetYaw - 3) || abs(ahrs->GetYaw()) > (targetYaw + 3))
    {
        alignYaw = ahrs->GetYaw();
        aTurn2PidOutput = m_aTurn2Controller.Calculate(abs(alignYaw), targetYaw);
        if (alignYaw < 0)
        {
            aTurn2PidOutput = aTurn2PidOutput * -1.0;
        }
        localAlign = false;
        R2Jesu_Drive(0.0, 0.0, aTurn2PidOutput);
        frc::SmartDashboard::PutNumber("aTurn2PidOutput", aTurn2PidOutput);
    }
    //if (((limelight_Table->GetNumber("tx",0.0) < -1.5) || (limelight_Table->GetNumber("tx",0.0) > 1.5)) && !((ahrs->GetYaw() > -177.0) && (ahrs->GetYaw() < 177.0)))
    if (((limelight_Table->GetNumber("tx",0.0) < -2.0) || (limelight_Table->GetNumber("tx",0.0) > 2.0)) && !(abs(ahrs->GetYaw()) < (targetYaw - 3) || abs(ahrs->GetYaw()) > (targetYaw + 3)))
    {
        aprilError = limelight_Table->GetNumber("tx",0.0);
        aprilCorrection = m_alignController.Calculate(aprilError, 0.0);
        localAlign = false;
        if (targetYaw == 0.0) {
            R2Jesu_Drive((aprilCorrection * -1.0), 0.0, 0.0);
        }
        else {
            R2Jesu_Drive(aprilCorrection, 0.0, 0.0);
        }
        frc::SmartDashboard::PutNumber("aprilCorrection", aprilCorrection);
    }
    frc::SmartDashboard::PutBoolean("local Align", localAlign);
    return localAlign;
    
}