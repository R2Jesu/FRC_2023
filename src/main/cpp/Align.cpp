#include "Robot.h"
bool localAlign;

bool Robot::R2Jesu_Align()
{
    printf("In align\n");
    localAlign = true;
    if ((ahrs->GetYaw() > -178.5) && (ahrs->GetYaw() < 178.5))
    {
        alignYaw = ahrs->GetYaw();
        aTurnPidOutput = m_aTurnController.Calculate(alignYaw, 180);
        //facingError = (fabs(alignYaw) - 180) * (fabs(alignYaw) / alignYaw);
        //facingCorrection = -.02 * facingError;
        //R2Jesu_Drive(0.0, 0.0, facingCorrection);
        frc::SmartDashboard::PutNumber("facingCorrection", aTurnPidOutput);
        //R2Jesu_Drive(0.0, 0.0, aTurnPidOutput);
        localAlign = false;
    }
    if ((limelight_Table->GetNumber("tx",0.0) < -1.5) || (limelight_Table->GetNumber("tx",0.0) > 1.5))
    {
        aprilError = limelight_Table->GetNumber("tx",0.0);
        aprilCorrection = m_alignController.Calculate(aprilError, 0.0);
        //R2Jesu_Drive(aprilCorrection, 0.0, 0.0);
        frc::SmartDashboard::PutNumber("aprilCorrection", aprilCorrection);
        localAlign = false;
    }
    R2Jesu_Drive(aprilCorrection, 0.0, aTurnPidOutput);
    return localAlign;
}