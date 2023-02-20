#include "Robot.h"
bool localAlign;

bool Robot::R2Jesu_Align()
{
    frc::SmartDashboard::PutNumber("align",v++);
    localAlign = true;
    aprilCorrection = 0.0;
    aTurn2PidOutput = 0.0;
    if ((ahrs->GetYaw() > -177.0) && (ahrs->GetYaw() < 177.0))
    {
        frc::SmartDashboard::PutNumber("Align YAW",j++);
        alignYaw = ahrs->GetYaw();
        aTurn2PidOutput = m_aTurn2Controller.Calculate(abs(alignYaw), 180);
        if (alignYaw < 0)
        {
            aTurn2PidOutput = aTurn2PidOutput * -1.0;
        }
        frc::SmartDashboard::PutNumber("aTurn2PidOutput", aTurn2PidOutput);
        localAlign = false;
    }
    if ((limelight_Table->GetNumber("tx",0.0) < -1.5) || (limelight_Table->GetNumber("tx",0.0) > 1.5))
    {
        frc::SmartDashboard::PutNumber("Align April",i++);
        aprilError = limelight_Table->GetNumber("tx",0.0);
        aprilCorrection = m_alignController.Calculate(aprilError, 0.0);
        frc::SmartDashboard::PutNumber("aprilCorrection", aprilCorrection);
        localAlign = false;
    }
    R2Jesu_Drive(aprilCorrection, 0.0, aTurn2PidOutput);
    frc::SmartDashboard::PutNumber("tx", limelight_Table->GetNumber("tx",0.0));
    frc::SmartDashboard::PutBoolean("local Align", localAlign);
    return localAlign;
    
}