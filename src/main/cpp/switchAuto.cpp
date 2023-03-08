#include "Robot.h"

void Robot::R2Jesu_SwitchAuto(void)
{
    printf("Starting Switch Auto\n");
    double balanceDirection = 0.0;
    if ((m_DriveEncoder1.GetPosition() > initialBackDistance) && !(initialBack))
    {
        m_SwerveDrive1.Set(-0.1);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(-0.1);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(-0.1);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(-0.1);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
    }
    else {
        if ((m_DriveEncoder1.GetPosition() <= initialBackDistance) && !(initialBack))
        {
            armSetPoint = 0.91;
            initialBack = true;
            m_DriveEncoder1.SetPosition(0.0);
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            m_SwerveTurn1.Set(0.0);
            m_SwerveTurn2.Set(0.0);
            m_SwerveTurn3.Set(0.0);
            m_SwerveTurn4.Set(0.0);
        }
    }

    if ((m_DriveEncoder1.GetPosition() > (-62.0 - autoOffset)) && initialBack && !(yDisplaceDone) && ((m_aprilSelected == 8.0) || m_aprilSelected == 3.0))
    {
        if ((((m_SwerveAnalog1.GetVoltage() * conversion1) > 5.0) && ((m_SwerveAnalog1.GetVoltage() * conversion1) < 175.0)) ||
        (((m_SwerveAnalog2.GetVoltage() * conversion2) > 5.0) && ((m_SwerveAnalog2.GetVoltage() * conversion2) < 175.0)) ||
        (((m_SwerveAnalog3.GetVoltage() * conversion3) > 5.0) && ((m_SwerveAnalog3.GetVoltage() * conversion3) < 175.0)) ||
        (((m_SwerveAnalog4.GetVoltage() * conversion4) > 5.0) && ((m_SwerveAnalog4.GetVoltage() * conversion4) < 175.0)))
        {
            m_SwerveDrive1.Set(0.0);
            pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
            m_SwerveTurn1.Set(pidOutput1);

            m_SwerveDrive2.Set(0.0);
            pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
            m_SwerveTurn2.Set(pidOutput2);

            m_SwerveDrive3.Set(0.0);
            pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
            m_SwerveTurn3.Set(pidOutput3);

            m_SwerveDrive4.Set(0.0);
            pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
            m_SwerveTurn4.Set(pidOutput4);
            wheelAngleCheck = false;
        } else
        {
            wheelAngleCheck = true;
        }
        if (wheelAngleCheck)
        { m_SwerveDrive1.Set(0.7);
         pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
         m_SwerveTurn1.Set(pidOutput1);
         m_SwerveDrive2.Set(0.7);
         pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
         m_SwerveTurn2.Set(pidOutput2);
         m_SwerveDrive3.Set(0.7);
         pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
         m_SwerveTurn3.Set(pidOutput3);
         m_SwerveDrive4.Set(0.7);
         pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
         m_SwerveTurn4.Set(pidOutput4);
        }
    }
    else
    {
        if ((m_DriveEncoder1.GetPosition() <= (-62.0 - autoOffset)) && initialBack && !(yDisplaceDone))
        {
            yDisplaceDone = true;
            m_DriveEncoder1.SetPosition(0.0);
            m_DriveEncoder3.SetPosition(0.0);
            R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }

    if ((m_DriveEncoder1.GetPosition() < (62.0 - autoOffset)) && initialBack && !(yDisplaceDone) && ((m_aprilSelected == 1.0) || m_aprilSelected == 6.0))
    {
        if ((((m_SwerveAnalog1.GetVoltage() * conversion1) > 5.0) && ((m_SwerveAnalog1.GetVoltage() * conversion1) < 175.0)) ||
        (((m_SwerveAnalog2.GetVoltage() * conversion2) > 5.0) && ((m_SwerveAnalog2.GetVoltage() * conversion2) < 175.0)) ||
        (((m_SwerveAnalog3.GetVoltage() * conversion3) > 5.0) && ((m_SwerveAnalog3.GetVoltage() * conversion3) < 175.0)) ||
        (((m_SwerveAnalog4.GetVoltage() * conversion4) > 5.0) && ((m_SwerveAnalog4.GetVoltage() * conversion4) < 175.0)))
        {
        m_SwerveDrive1.Set(0.0);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(0.0);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(0.0);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(0.0);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
        m_SwerveTurn4.Set(pidOutput4);
        wheelAngleCheck = false;
        } else
        {
        wheelAngleCheck = true;
        }
        if (wheelAngleCheck)
        {
        m_SwerveDrive1.Set(-0.7);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(-0.7);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(-0.7);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(-0.7);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
        m_SwerveTurn4.Set(pidOutput4);
        }
    }
    else
    {
        if ((m_DriveEncoder1.GetPosition() >= (62.0 - autoOffset)) && initialBack && !(yDisplaceDone))
        {
            yDisplaceDone = true;
            m_DriveEncoder1.SetPosition(0.0);
            m_DriveEncoder3.SetPosition(0.0);
            R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }

    if (initialBack && !(yDisplaceDone) && ((m_aprilSelected == 7.0) || m_aprilSelected == 2.0) && !(yDisplaceDone))
    {
            yDisplaceDone = true;
            m_DriveEncoder1.SetPosition(0.0);
            m_DriveEncoder3.SetPosition(0.0);
            R2Jesu_Drive(0.0, 0.0, 0.0);
    }

    if (yDisplaceDone && !(turnt) && abs(ahrs->GetYaw()) > 5.0 ) {
       R2Jesu_Drive(0.0, 0.0, 0.8);
    }
    else{
        if (yDisplaceDone && !(turnt)) {
           turnt = true;
           m_DriveEncoder1.SetPosition(0.0);
           m_DriveEncoder3.SetPosition(0.0);
           R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }
    
    if ((m_DriveEncoder3.GetPosition() < 55.0) && !(hasRun) && yDisplaceDone && turnt)
    {
        m_SwerveDrive1.Set(0.3);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(0.3);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(0.3);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(0.3);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
        
    }
    else {
        if ((m_DriveEncoder3.GetPosition() >= 55.0) && yDisplaceDone && !(hasRun) && turnt)
        {
            hasRun = true;
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            m_SwerveTurn1.Set(0.0);
            m_SwerveTurn2.Set(0.0);
            m_SwerveTurn3.Set(0.0);
            m_SwerveTurn4.Set(0.0);
        }
    }

    if ((ahrs->GetPitch() >= 4.0 || ahrs->GetPitch() <= -4.0) && hasRun)
    {
        balanceDirection = 270.0;
        switchPidOutput = m_switchController.Calculate((ahrs->GetPitch()), 0.0);
        m_SwerveDrive1.Set(switchPidOutput);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), balanceDirection);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(switchPidOutput);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), balanceDirection);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(switchPidOutput);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), balanceDirection);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(switchPidOutput);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), balanceDirection);
        m_SwerveTurn4.Set(pidOutput4);

    }
    else
    {
        if (hasRun) {
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            m_SwerveTurn1.Set(0.0);
            m_SwerveTurn2.Set(0.0);
            m_SwerveTurn3.Set(0.0);
            m_SwerveTurn4.Set(0.0);
    
    
            pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 45.0);
            m_SwerveTurn1.Set(pidOutput1);
            pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 135.0);
            m_SwerveTurn2.Set(pidOutput2);
            pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 45.0);
            m_SwerveTurn3.Set(pidOutput3);
            pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 135.0);
            m_SwerveTurn4.Set(pidOutput4);
        }
    } 
} 